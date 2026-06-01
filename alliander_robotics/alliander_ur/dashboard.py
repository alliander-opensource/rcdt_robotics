import argparse
import socket


class Dashboard:
    """Class to send commands to the UR robot's dashboard server."""

    def __init__(self):
        """Initialize."""
        self.ip = "172.16.0.2"
        self.port = 29999

    def get_mode(self, verbose: bool = True) -> str:
        """Get the mode of the robot arm.

        Args:
            verbose (bool, optional): Whether to print the response.

        Returns:
            str: The mode of the robot arm.
        """
        response = self.send_command("robotmode")
        if verbose:
            print(response)
        return response

    def power_on(self, verbose: bool = True) -> str:
        """Power on the robot arm.

        Args:
            verbose (bool, optional): Whether to print the response.

        Returns:
            str: The mode of the robot arm.
        """
        response = self.send_command("power on")
        if verbose:
            print(response)
        return response

    def power_off(self, verbose: bool = True) -> str:
        """Power off the robot arm.

        Args:
            verbose (bool, optional): Whether to print the response.

        Returns:
            str: The mode of the robot arm.
        """
        response = self.send_command("power off")
        if verbose:
            print(response)
        return response

    def release(self, verbose: bool = True) -> str:
        """Release the brakes of the robot arm.

        Args:
            verbose (bool, optional): Whether to print the response.

        Returns:
            str: The mode of the robot arm.
        """
        response = self.send_command("brake release")
        if verbose:
            print(response)
        return response

    def shutdown(self, verbose: bool = True) -> str:
        """Shutdown the controller.

        Args:
            verbose (bool, optional): Whether to print the response.

        Returns:
            str: The mode of the robot arm.
        """
        mode = self.get_mode(verbose=False)
        if mode != "Robotmode: POWER_OFF":
            print(
                "Robot arm is not powered off. Please power off the arm before shutdown of the controller."
            )
            return ""
        response = self.send_command("shutdown")
        if verbose:
            print(response)
        return response

    def send_command(self, command: str) -> str:
        """Send a command to the UR robot's dashboard server and return the feedback.

        Args:
            command (str): The command to send to the dashboard server.

        Returns:
            str: The feedback received from the dashboard server.
        """
        try:
            with socket.create_connection((self.ip, self.port), timeout=2) as s:
                # Receive the server's initial connection banner
                _initial_msg = s.recv(1024).decode("utf-8")

                # Send the desired command, ensuring the \n termination
                message = f"{command}\n"
                s.sendall(message.encode("utf-8"))

                # Read and return the server's feedback
                feedback = s.recv(1024).decode("utf-8").strip()
                return feedback

        except Exception as e:
            return f"Error connecting to the robot: {e}"


if __name__ == "__main__":
    dashboard = Dashboard()
    parser = argparse.ArgumentParser(
        description="Send commands to the UR robot's dashboard server."
    )

    parser.add_argument(
        "--mode",
        required=False,
        action="store_true",
        help="Get the mode of the robot arm.",
    )
    parser.add_argument(
        "--on",
        required=False,
        action="store_true",
        help="Power on the robot arm.",
    )
    parser.add_argument(
        "--release",
        required=False,
        action="store_true",
        help="Release the brakes of the robot arm.",
    )
    parser.add_argument(
        "--off",
        required=False,
        action="store_true",
        help="Power off the robot arm.",
    )
    parser.add_argument(
        "--shutdown",
        required=False,
        action="store_true",
        help="Shutdown the controller.",
    )
    parser.add_argument(
        "--ip",
        required=False,
        default="",
        help=f"The IP address of the controller. If not provided, the default IP address of {dashboard.ip} will be used.",
    )

    args = parser.parse_args()

    if args.ip:
        dashboard.ip = args.ip
    if args.mode:
        dashboard.get_mode()
    elif args.on:
        dashboard.power_on()
    elif args.off:
        dashboard.power_off()
    elif args.release:
        dashboard.release()
    elif args.shutdown:
        dashboard.shutdown()
