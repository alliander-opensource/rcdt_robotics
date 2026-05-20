#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
"""
OpenVLA Model Manager
Handles model loading, caching, and quantization with performance monitoring
"""
# from anyio.functools import cache

import logging
import os
import time
import psutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Union

import numpy as np
import re
import torch
import yaml
from accelerate import infer_auto_device_map, dispatch_model
from transformers import AutoModelForVision2Seq, AutoProcessor, BitsAndBytesConfig, AutoConfig

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class ModelMetrics:
    """Track model performance metrics."""
    load_time: float = 0.0
    inference_times: list = None
    memory_usage_mb: float = 0.0
    successful_inferences: int = 0
    failed_inferences: int = 0

    def __post_init__(self):
        if self.inference_times is None:
            self.inference_times = []

    def avg_inference_time(self) -> float:
        return sum(self.inference_times) / len(self.inference_times) if self.inference_times else 0.0

    def fps(self) -> float:
        avg_time = self.avg_inference_time()
        return 1.0 / avg_time if avg_time > 0 else 0.0


class ModelManager:
    """Manages OpenVLA model lifecycle with performance tracking."""

    def __init__(self, config_path: str = "config/model_config.yaml"):
        self.config = self._load_config(config_path)
        self.model = None
        self.processor = None
        self.metrics = ModelMetrics()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        logger.info(f"Initialized ModelManager on device: {self.device}")

    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)

    def _get_quantization_config(self) -> Optional[BitsAndBytesConfig]:
        """Create quantization configuration."""
        if not self.config["model"]["quantization"]["enabled"]:
            return None

        quant_config = self.config["model"]["quantization"]

        return BitsAndBytesConfig(
            load_in_4bit=(quant_config["bits"] == 4),
            load_in_8bit=(quant_config["bits"] == 8),
            bnb_4bit_compute_dtype=getattr(torch, quant_config["compute_dtype"]),
            bnb_4bit_quant_type=quant_config["method"],
            bnb_4bit_use_double_quant=quant_config["double_quant"],
            #llm_int8_enable_fp32_cpu_offload=True  # Important! I need CPU offloading because VRAM is too small to load the full model
        )
    
    
    # def _parse_generated_action(self, text: str) -> np.ndarray:
    #     """
    #     Extract numeric values from generated text.
    #     Works for formats like:
    #     - "[0.1, -0.2, 0.3]"
    #     - "x=0.1 y=-0.2 z=0.3"
    #     - "move to 0.1, -0.2, 0.3"
    #     """
    #     try:
    #         # Extract all numbers (floats and ints)
    #         values = re.findall(r"[-+]?\d*\.\d+|\d+", text)

    #         if not values:
    #             raise ValueError("No numbers found in generated output")

    #         action = np.array([float(v) for v in values], dtype=np.float32)

    #         # Optional: enforce expected dimension
    #         expected_dim = self.config["tasks"][self.current_task]["action_dims"]
    #         if len(action) != expected_dim:
    #             self.node_logger.warn(
    #                 f"Expected {expected_dim} dims, got {len(action)}"
    #             )

    #         return action

    #     except Exception as e:
    #         logger.error(f"Failed to parse generated action: {e}")
    #         logger.error(f"Raw generated output: {text}")

    #         # fallback: safe output
    #         action_dims = self.config["tasks"][self.current_task]["action_dims"]
    #         return np.zeros(action_dims, dtype=np.float32)


    # THIS IS THE ORIGINAL load_model() FUNCTION! with added logger.info statements
    def load_model(self) -> bool:
        """Load the OpenVLA model with quantization
        Returns True if successful, False otherwise.
        """
        try:
            start_time = time.time()
            model_name = self.config['model']['name']

            logger.info(f"Loading model: {model_name}")
            logger.info(f"Quantization: {self.config['model']['quantization']['enabled']}")

            # Load processor
            self.processor = AutoProcessor.from_pretrained(
                model_name,
                trust_remote_code=True
            )

            # Get quantization config
            quantization_config = self._get_quantization_config()

            # Load model
            self.model = AutoModelForVision2Seq.from_pretrained(
                model_name,
                quantization_config=quantization_config,
                torch_dtype=getattr(torch, self.config['model']['inference']['torch_dtype']),
                device_map="auto",
                trust_remote_code=True,
                cache_dir=self.config['model']['cache_dir']
            )

            # Compile model if enabled
            if self.config['model']['inference']['compile']:
                logger.info("Compiling model with torch.compile()...")
                compile_mode = self.config['model']['inference']['compile_mode']
                self.model = torch.compile(self.model, mode=compile_mode)

            logger.info(type(self.model))

            # Track metrics
            self.metrics.load_time = time.time() - start_time
            self.metrics.memory_usage_mb = torch.cuda.memory_allocated() / 1024**2
            
            devices = set(p.device.type for p in self.model.parameters())
            logger.info(f"Devices used: {devices}")

            # logger.info(f"Layer-device mapping: {getattr(self.model, "hf_device_map", "No device map")}")

            self.print_model_device_distribution()
            
            # # CPU memory (process)
            # process = psutil.Process(os.getpid())
            # cpu_mem = process.memory_info().rss / 1024**2
            # print(f"CPU RAM used by process: {cpu_mem:.2f} MB")


            logger.info(f"Model loaded successfully in {self.metrics.load_time:.2f}s")
            logger.info(f"Memory usage: {self.metrics.memory_usage_mb:.2f} MB")

            return True

        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            return False

    # cache_dir = self.config["model"]["cache_dir"]
    # offload_folder=f"{cache_dir}/offload",

    # def load_model(self) -> bool:
    #     try:
    #         start_time = time.time()
    #         model_name = self.config["model"]["name"]
    #         torch_dtype = getattr(torch, self.config["model"]["inference"]["torch_dtype"])
    #         cache_dir = self.config["model"]["cache_dir"]

    #         self.processor = AutoProcessor.from_pretrained(model_name, trust_remote_code=True)

    #         # We use a tight GPU limit to leave room for the Vision Tower 
    #         # and ensure language embeddings/head stay together on CPU.
    #         max_memory = {0: "2.8GB", "cpu": "14GB"}

    #         logger.info("Loading OpenVLA (Final Attempt: low_cpu_mem_usage=False)...")
    #         self.model = AutoModelForVision2Seq.from_pretrained(
    #             model_name,
    #             quantization_config=self._get_quantization_config(),
    #             torch_dtype=torch_dtype,
    #             device_map="auto",
    #             max_memory=max_memory,
    #             trust_remote_code=True,
    #             # CRITICAL: Must be False to avoid meta-tensor .item() crash
    #             low_cpu_mem_usage=False, 
    #             offload_folder=f"{cache_dir}/offload",
    #             cache_dir=cache_dir
    #         )

    #         if hasattr(self.model, "tie_weights"):
    #             self.model.tie_weights()

    #         self.metrics.load_time = time.time() - start_time
    #         self.metrics.memory_usage_mb = torch.cuda.memory_allocated() / 1024**2
    #         logger.info(f"Model successfully loaded! GPU: {self.metrics.memory_usage_mb:.2f} MB")
    #         return True

    #     except Exception as e:
    #         # If this still gives exit code -9, your system RAM is too full.
    #         # Close all other processes or increase swap space.
    #         logger.error(f"Failed to load model: {e}")
    #         return False

    # def load_model_old(self) -> bool:
    #     """Load the OpenVLA model with quantization.

    #     Returns:
    #         Boolean that returns True if successful, False otherwise
    #     """
    #     try:
    #         start_time = time.time()
    #         model_name = self.config["model"]["name"]

    #         logger.info(f"Loading model: {model_name}")
    #         logger.info(f"Quantization: {self.config['model']['quantization']['enabled']}")

    #         # Load processor
    #         self.processor = AutoProcessor.from_pretrained(
    #             model_name,
    #             trust_remote_code=True
    #         )

    #         # Get quantization config
    #         quantization_config = self._get_quantization_config()

    #         cache_dir = self.config["model"]["cache_dir"]

    #         # Load model
    #         self.model = AutoModelForVision2Seq.from_pretrained(
    #             model_name,
    #             quantization_config=quantization_config,
    #             torch_dtype=getattr(torch, self.config["model"]["inference"]["torch_dtype"]),
    #             device_map=None,
    #             trust_remote_code=True,
    #             cache_dir=cache_dir
    #         )

    #         # 4. MANUALLY tie weights now that the model is loaded in RAM
    #         if hasattr(self.model, "tie_weights"):
    #             logger.info("Manually tying weights...")
    #             self.model.tie_weights()

    #         # 5. Manually calculate the device map
    #         # We specify 'no_split_module_classes' to ensure transformer blocks stay intact
    #         max_memory: Dict[Union[int, str], Union[int, str]] = {
    #             0: "3.5GB",   # GPU limit
    #             "cpu": "20GB"
    #         }

    #         # OpenVLA is Llama-based; LlamaDecoderLayer should not be split
    #         no_split_modules = ["LlamaDecoderLayer", "ClipViTBlock", "PaliGemmaBlock"] 

    #         device_map = infer_auto_device_map(
    #             self.model,
    #             max_memory=max_memory,
    #             no_split_module_classes=no_split_modules
    #         )

    #         # 6. Dispatch the model to the devices
    #         logger.info("Dispatching model to GPU/CPU...")
    #         self.model = dispatch_model(
    #             self.model,
    #             device_map=device_map,
    #             offload_dir="offload"
    #         )

    #         # # Added to fix 'model weights are not tied' error
    #         # if hasattr(self.model, "tie_weights"):
    #         #     self.model.tie_weights()

    #         # device_map = infer_auto_device_map(
    #         #     self.model,
    #         #     max_memory={
    #         #         0: "3.5GB",
    #         #         "cpu": "20GB"
    #         #     }
    #         # )

    #         # self.model = dispatch_model(
    #         #     self.model,
    #         #     device_map=device_map,
    #         #     offload_dir="/models/offload"
    #         # )

    #         # # Compile model if enabled
    #         # if self.config["model"]["inference"]["compile"]:
    #         #     logger.info("Compiling model with torch.compile()...")
    #         #     compile_mode = self.config["model"]["inference"]["compile_mode"]
    #         #     self.model = torch.compile(self.model, mode=compile_mode)

    #         # Track metrics
    #         self.metrics.load_time = time.time() - start_time
    #         self.metrics.memory_usage_mb = torch.cuda.memory_allocated() / 1024**2

    #         logger.info(f"Model loaded successfully in {self.metrics.load_time:.2f}s")
    #         logger.info(f"Memory usage: {self.metrics.memory_usage_mb:.2f} MB")

    #         return True

    #     except Exception as e:
    #         logger.error(f"Failed to load model: {e}")
    #         return False

    def infer(self, image, prompt: str, task: str = "default") -> Optional[Dict[str, Any]]:
        """Run inference on image with given prompt.

        Args:
            image: PIL Image or numpy array
            prompt: Text prompt for the task
            task: Task type (used for post-processing)

        Returns:
            Dictionary with action prediction and metadata
        """
        if self.model is None or self.processor is None:
            logger.error("Model not loaded. Call load_model() first.")
            return None

        try:
            start_time = time.time()

            # Prepare inputs
            model_dtype = getattr(torch, self.config["model"]["inference"]["torch_dtype"])
            inputs = self.processor(prompt, image, return_tensors="pt").to(self.device, dtype=model_dtype)

            # Run inference
            try:
                action = self.model.predict_action(
                    **inputs,
                    unnorm_key="nyu_franka_play_dataset_converted_externally_to_rlds",   # ToDo: Check unnorm_keys; do I need a different one? Now set to the only franka unnorm key available
                    do_sample=False
                )
                # logger.info(f"Raw action array: {action}")
                # logger.info(f"Action length: {len(action)}")
                action_str = f'Coordinates: [{", ".join([f"{x:.3f}" for x in action.tolist()])}]'
            except Exception as e:
                logger.warning(f"predict_action failed, returning None: {e}")
                return None

            # Track metrics
            inference_time = time.time() - start_time
            self.metrics.inference_times.append(inference_time)
            self.metrics.successful_inferences += 1

            result = {
                "action": action,
                "inference_time": inference_time,
                "prompt": prompt,
                "task": task,
                "timestamp": time.time(),
                "action_str": action_str
            }

            logger.info(f"Inference completed in {inference_time:.3f}s")
            return result

        except Exception as e:
            logger.error(f"Inference failed: {e}")
            self.metrics.failed_inferences += 1
            return None

    def get_metrics(self) -> Dict[str, Any]:
        """Return the current performance metrics.

        Return:
            Dictionary containing the performance metrics
        """
        return {
            "load_time": self.metrics.load_time,
            "avg_inference_time": self.metrics.avg_inference_time(),
            "fps": self.metrics.fps(),
            "memory_usage_mb": self.metrics.memory_usage_mb,
            "total_inferences": self.metrics.successful_inferences + self.metrics.failed_inferences,
            "successful": self.metrics.successful_inferences,
            "failed": self.metrics.failed_inferences,
            "success_rate": (
                self.metrics.successful_inferences /
                (self.metrics.successful_inferences + self.metrics.failed_inferences)
                if (self.metrics.successful_inferences + self.metrics.failed_inferences) > 0
                else 0.0
            )
        }

    def print_metrics(self) -> None:
        """Print formatted metrics."""
        metrics = self.get_metrics()
        print("\n" + "=" * 50)
        print("MODEL PERFORMANCE METRICS")
        print("=" * 50)
        print(f"Load Time:           {metrics['load_time']:.2f}s")
        print(f"Avg Inference Time:  {metrics['avg_inference_time'] * 1000:.1f}ms")
        print(f"FPS:                 {metrics['fps']:.2f}")
        print(f"Memory Usage:        {metrics['memory_usage_mb']:.2f} MB")
        print(f"Total Inferences:    {metrics['total_inferences']}")
        print(f"Success Rate:        {metrics['success_rate'] * 100:.1f}%")
        print("=" * 50 + "\n")

    
    def print_model_device_distribution(self):
        """
        Prints how much of the model is on GPU vs CPU based on parameter count.

        Args:
            model: PyTorch model
        """
        gpu_params = 0
        cpu_params = 0
        total_params = 0

        for p in self.model.parameters():
            num = p.numel()
            total_params += num

            if p.device.type == "cuda":
                gpu_params += num
            else:
                cpu_params += num

        if total_params == 0:
            print("Model has no parameters.")
            return

        gpu_pct = 100 * gpu_params / total_params
        cpu_pct = 100 * cpu_params / total_params

        logger.info(f"Total parameters: {total_params:,}")
        logger.info(f"GPU parameters:   {gpu_params:,} ({gpu_pct:.2f}%)")
        logger.info(f"CPU parameters:   {cpu_params:,} ({cpu_pct:.2f}%)")

