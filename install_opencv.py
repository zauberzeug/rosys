import re
import subprocess


def add_package(package_name: str) -> None:
    print(f'Adding package {package_name}')
    subprocess.run(['poetry', 'add', package_name], check=True)
    print(f'Package {package_name} added')


def get_installed_version(package_name: str) -> str:
    result = subprocess.run(['poetry', 'show', package_name], capture_output=True, text=True, check=True)
    try:
        return re.search(r'version\s+:\s+(\S+)', result.stdout).group(1)
    except AttributeError as e:
        raise ValueError(f'Could not find version of {package_name} in {result.stdout}') from e


def main() -> None:
    opencv_package = 'opencv-python'
    contrib_package = 'opencv-contrib-python-headless'

    add_package(opencv_package)

    opencv_version = get_installed_version(opencv_package)
    print(f'Installed version of {opencv_package}: {opencv_version}')

    add_package(f'{contrib_package}=={opencv_version}')


if __name__ == '__main__':
    main()
