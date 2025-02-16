# Firmware Engineer Position Test

This repo contains the hello_world sample from the Zephyr project using the T2 topology.

## Requirements

In order to compile and flash this code you will need to have all the required tools to build a Zephyr project.
If you don't have it, visit the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)

## Usage

### Create the workspace

Create a folder for the workspace where this repo as well as the full version of the zephyr-rtos will be installed.

```bash
mkdir our_app_workspace

cd our_app_workspace
```

### Activate the virtual environment where west is installed

```bash
source ~/zephyrproject/.venv/bin/activate
```

### Initialize the project

```bash
west init -m <this_repo_url>

west update
```
## Build the hello_world sample

In order to test if the workspace is ok try to build the code for the nRF9160 dev kit.

 ```bash
 cd app

 west build -p -b nrf9160dk/nrf9160
 ```
