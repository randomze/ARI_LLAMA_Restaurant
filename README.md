# Local LLAMA Restaurant

In this repository you can find an example ROS package for having the ARI act as a restaurant host.

There are two main components:

- `RODGER`, which is the local LLAMA-powered chat bot. This is in charge of handling conversations with the user.
- `cloud_interaction`, which is the ROS package responsible for interfacing the ARI with `RODGER` and handling movement.

The prerequisites for running this package can be installed by running `pip install -r requirements.txt`.
It is meant to be run on a GPU-powered laptop, since the ARI does not have a GPU.

Running this package requires having a map of the environment and two POIs (Points of Interest), named `queue` and `table_1`.
Read the documentation for a guide on how to do this.

## Setup

Start by cloning this repository by running:

```bash
git clone --recurse-submodules https://github.com/randomze/ARI_LLAMA_Restaurant.git
```

## Running the demo

The demo can be ran with:

```bash
./demo.sh
```

You can look through the `demo.sh` file, which is documented, to understand how the different components are started.
