# SO-101 in MuJoCo without ROS, on the CPU: the tests with rendering, and a trained policy's evaluation.
#
#   docker build -t so101_lerobot .
#   docker run --rm so101_lerobot                                  # the so101_mujoco tests
#   docker run --rm -v <pretrained_model dir>:/policy:ro so101_lerobot \
#       so101_mujoco_eval /policy --device cpu --episodes 5        # a policy trained with lerobot-train
#
# On a slow or unreliable link, download the wheels first and build from them (nothing is downloaded then):
#   docker build --build-context wheels=<dir with the wheels> -t so101_lerobot .
# The hand teleoperation (webcam, window) and the training (GPU) run natively: see the README.

# The local wheels: an empty stage unless `--build-context wheels=<dir>` replaces it
FROM scratch AS wheels

FROM python:3.12-slim-bookworm

# MuJoCo renders off-screen through EGL with Mesa's software renderer: no GPU and no display; glib is for OpenCV
RUN apt-get update \
    && apt-get install -y --no-install-recommends libegl1 libegl-mesa0 libgl1 libgles2 libgl1-mesa-dri libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/so101_lerobot
ENV PIP_RETRIES=10 PIP_DEFAULT_TIMEOUT=60 PIP_ROOT_USER_ACTION=ignore PIP_DISABLE_PIP_VERSION_CHECK=1
# torch built for the CPU, without the lock's CUDA packages (several GB); a step of its own, kept once it succeeded
RUN --mount=type=bind,from=wheels,target=/wheels --mount=type=cache,target=/root/.cache/pip \
    if ls /wheels/*.whl > /dev/null 2>&1; then source="--no-index --find-links /wheels"; \
    else source="--index-url https://download.pytorch.org/whl/cpu"; fi \
    && pip install $source torch==2.10.0 torchvision==0.25.0
# the rest at the venv's pinned versions
COPY requirements-lock.txt ./
RUN --mount=type=bind,from=wheels,target=/wheels --mount=type=cache,target=/root/.cache/pip \
    grep -v -E '^(torch|torchvision|nvidia-|triton|cuda-)' requirements-lock.txt > /tmp/constraints.txt \
    && if ls /wheels/*.whl > /dev/null 2>&1; then source="--no-index --find-links /wheels"; else source=""; fi \
    && pip install $source -c /tmp/constraints.txt "lerobot[dataset]==0.7.0" so101-nexus==0.11.0 pytest
# so101-nexus writes the scene's XML next to its assets at run time: writable for any user (`docker run --user`)
RUN chmod -R a+w "$(python -c 'import os, so101_nexus; print(os.path.dirname(so101_nexus.__file__))')/assets"

COPY so101_hand so101_hand
COPY so101_mujoco so101_mujoco
RUN pip install --no-cache-dir --no-deps -e so101_hand -e so101_mujoco

# the evaluation's CSV goes to $DATA_DIR/so101_mujoco; nothing is fetched from the Hugging Face Hub; HOME and USER
# let any user run it (`--user "$(id -u):$(id -g)"` keeps the results the caller's): torch asks for a user name
ENV MUJOCO_GL=egl DATA_DIR=/data HF_HUB_OFFLINE=1 HOME=/tmp USER=so101
CMD ["python", "-m", "pytest", "-q", "so101_mujoco"]
