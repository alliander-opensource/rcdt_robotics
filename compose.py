import argparse
import yaml
import subprocess
import os


def get_image_tag():
    try:
        branch_name = subprocess.check_output(
            ['git', 'rev-parse', '--abbrev-ref', 'HEAD'],
            universal_newlines=True,
            stderr=subprocess.PIPE
        ).strip()
    except subprocess.CalledProcessError:
        print("Warning: not in a Git repository. Defaulting to 'latest'.")
        return "latest"
    except FileNotFoundError:
        print("Warning: 'git' command not found. Defaulting to 'latest'.")
        return "latest"
    
    if branch_name == 'main':
        return 'latest'
    return branch_name


def compose_and_write(arch: str, platforms: list, output_file: str = "docker-compose.yml"):
    merged_compose = {}
    image_tag = get_image_tag()

    print(f"--- Using image tag {image_tag} ---")

    for platform in platforms:
        filename = f"rcdt_{platform}/{arch}.yml"

        if not os.path.exists(filename):
            print(f"Warning: file {filename} not found. Skipping.")
            continue

        print(f"Merging {filename}")

        with open(filename, 'r') as f:
            content = yaml.safe_load(f)

            if not content:
                continue

            if 'services' in content:
                if 'services' not in merged_compose:
                    merged_compose['services'] = {}
                merged_compose['services'].update(content['services'])

            for key, value in content.items():
                if key != 'services' and key not in merged_compose:
                    merged_compose[key] = value

    if not merged_compose:
        print("Error: no compose files were merged. Aborting.")
        return
    
    if 'services' in merged_compose:
        print("Replacing ${IMAGE_TAG} in service images...")
        for _, service_config in merged_compose['services'].items():
            if 'image' in service_config and '${IMAGE_TAG}' in service_config['image']:
                original_image = service_config['image']
                service_config['image'] = original_image.replace('${IMAGE_TAG}', image_tag)

    print(f"\nWriting final compose file to **{output_file}**")

    class Dumper(yaml.Dumper):
        def increase_indent(self, flow: bool = False, indentless: bool = False) -> None:
            return super().increase_indent(flow, indentless)

    with open(output_file, 'w') as f:
        yaml.safe_dump(merged_compose, f, default_flow_style=False, sort_keys=False)

    print("Done!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Creates a combined docker-compose.yml file from platform-specific composes.")

    parser.add_argument('--arch', required=True, choices=['amd64', 'arm64'], help="Target architecture (amd64 or arm64).")

    parser.add_argument('--platforms', required=True, nargs='+', help="List of platform components to include (e.g. husarion franka simulation). Names need to match with the rcdt_ directories in the repository root.")

    args = parser.parse_args()

    compose_and_write(args.arch, args.platforms)
