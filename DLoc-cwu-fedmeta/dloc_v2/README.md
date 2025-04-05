# DLoc++ Codebase

## Setup
### On CUDA Server
If you are on CUDA server, the docker container is likely set up for you and already running. Verify with the following command:
```
docker ps | grep dloc
```
If docker container is already running, to enter the container run:
```
docker exec -it dloc_docker /bin/bash
```
### New Setup
If the docker container is not running, you can set it up with few steps.
1. Start the docker container
    ```
    docker run --gpus all --shm-size=64g -d -v /media:/media --name dloc_docker pytorchlightning/pytorch_lightning:base-cuda-py3.9-torch1.11-cuda11.3.1 tail -f /dev/null
    ```
    > NOTE: You can use any base Docker image as long as it includes PyTorch with CUDA 11.3 support, which matches the CUDA version on the server.
2. Enter the container
    ```
    docker exec -it dloc_docker /bin/bash
    ```
3. Install necessary dependencies
    ```
    pip install pytorch-lightning==1.9.1
    pip install h5py==3.11.0
    pip install torchvision==0.12.0+cu113 --extra-index-url https://download.pytorch.org/whl/cu113
    pip install python-dotenv==1.0.1
    pip install comet_ml==3.35.6
    ```
    > NOTE: If you have a different PyTorch version, please ensure `pytorch-lightning` and `torchvision` versions are compatible with your PyTorch version.


## Training
The main training and validation script is [main.py](./main.py).
```
python main.py
```

## Comet Credentials
If you use Comet ML, consider saving your credentials as environment variables to avoid committing sensitive information like API keys to GitHub.
1. Create `.env` file:
    ```
    cd dloc_v2
    touch .env
    ```
2. Inside `.env` file, specify the API key and workspace name:
    ```
    COMET_API_KEY=<your_api_key>
    COMET_WORKSPACE=<your_workspace_name>
    ```
    > The `.env` file is included in the `.gitignore` file, ensuring that its content is never committed to GitHub.

The `load_dotenv()` function inside the `main` function of [main.py](./main.py) will automatically set environment variables according to the `.env` file.
