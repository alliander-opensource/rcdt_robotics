<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# Grasping

Grasping is an import but challenging skill of a robot. To integrate this skill to our robots, we require a tool that can define the best grasp position for our robot gripper, to grasp an object of interest. We define three different strategies to define the best grasp position:

**Logic based**\
The logic based strategy defines 'rules' that should lead to grasping an object successfully. This strategy can be useful when the object to grasp is constant. For example: to grasp bricks, one might introduce rules like 'grasp the brick as much centered as possible' and 'place the fingers always on the long sides of the brick'. The great disadvantage of this method is that it doesn't scale when the objects to grasp differ or are unknown. It is impossible to define rules for every possible object.

**Generation based**\
The generation based strategy uses a trained AI model to sample grasp poses on a given object, followed by a scoring and filtering to select the best poses. Creation of the required models is complex and very computational expensive, but fortunately are some models shared, for example with research papers. Two popular model published on GitHub are [GraspNet](https://github.com/graspnet/graspnet-baseline) ([AnyGrasp](https://arxiv.org/pdf/2212.08333)) and [GrasGen](https://github.com/NVlabs/GraspGen) ([paper](https://arxiv.org/pdf/2507.13097)).

**Vision Language Action based**\
The vision language action based strategy requires an AI model that integrates vision, language and actions to execute tasks. With these models, one could theoretically give a command like 'pull the red handle'. A model like [DexGraspVLA](https://github.com/Psi-Robot/DexGraspVLA) should create an action list to bring the visioned state to the commanded state. This is a continuous process of performing actions and checking the resulting visional feedback, trying to get closer to the commanded state.

Since the logic-based strategy is not scalable, while VLA is still very complex and controls the whole action strategy of the robot, we decided to focus on the generation based strategy.

## Integration of a generation based method

:::{mermaid} ../diagrams/grasping.mmd
:::
