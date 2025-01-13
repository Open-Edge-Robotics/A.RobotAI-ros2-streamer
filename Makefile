# Makefile for building and launching a ROS2_PATH Python package

# Package name
PACKAGE_NAME=ros_stream_server
NODE_NAME=stream_server

ROS2_PATH:=/opt/ros/$(ROS_DISTRO)

PYTHON=python3
SHELL:=/bin/bash
CURRENT_DIR := $(shell pwd)

# Docker variables
DOCKER_ORG = lgecloudroboticstask
DOCKER_IMAGE = fl
DOCKER_TAG := ros-streamer-$(shell date +%Y%m%d)

all: build

dep:
	@echo "Installing dependencies..."
	$(PYTHON) -m pip install -r requirements.txt
	. $(ROS2_PATH)/setup.bash && rosdep install --from-paths src --ignore-src -r -y

build: dep
	@echo "Building package..."
	. $(ROS2_PATH)/setup.bash && colcon build

build-symlink-install: dep
	@echo "Building package with symlink install..."
	. $(ROS2_PATH)/setup.bash && colcon build --symlink-install

run: build
	@echo "Running package..."
	. $(ROS2_PATH)/setup.bash && . install/setup.bash && ros2 run $(PACKAGE_NAME) $(NODE_NAME)

launch: build
	. $(ROS2_PATH)/setup.bash && . install/setup.bash && ros2 launch $(PACKAGE_NAME) $(NODE_NAME).launch.py

view:
	$(PYTHON) viewer/viewer.py

docker-build:
	@echo Building Streamer Docker image...
	cd src/ros_stream_server && docker build -t $(DOCKER_ORG)/$(DOCKER_IMAGE):$(DOCKER_TAG) .
	@echo Updating Kubernetes deployment...
	@sed -i 's|image: $(DOCKER_ORG)/$(DOCKER_IMAGE):.*|image: $(DOCKER_ORG)/$(DOCKER_IMAGE):$(DOCKER_TAG)|' src/ros_stream_server/kube/deployment.yaml
	@echo Streamer Docker image built.

docker-push: docker-build
	@echo Pushing Streamer Docker image...
	@docker push $(DOCKER_ORG)/$(DOCKER_IMAGE):$(DOCKER_TAG)
	@echo Streamer Docker image pushed.

podman-build:
	@echo Building Streamer Podman image...
	cd src/ros_stream_server && podman build -t $(DOCKER_ORG)/$(DOCKER_IMAGE):$(DOCKER_TAG) .
	@echo Updating Kubernetes deployment...
	@sed -i 's|image: $(DOCKER_ORG)/$(DOCKER_IMAGE):.*|image: $(DOCKER_ORG)/$(DOCKER_IMAGE):$(DOCKER_TAG)|' src/ros_stream_server/kube/ros-streamer-deployment.yaml
	@echo Streamer Podman image built.

podman-push: podman-build
	@echo Pushing Streamer Podman image...
	@podman push docker.io/$(DOCKER_ORG)/$(DOCKER_IMAGE):$(DOCKER_TAG)
	@echo Streamer Podman image pushed.

docker-clean:
	@echo Cleaning up Docker images...
ifeq ($(shell docker images -q $(DOCKER_ORG)/$(DOCKER_IMAGE):$(DOCKER_TAG)),)
	@echo No Docker image found.
else
	docker rmi $(DOCKER_ORG)/$(DOCKER_IMAGE):$(DOCKER_TAG)
endif
	@echo Docker images cleaned.

podman-clean:
	@echo Cleaning up Podman images...
ifeq ($(shell podman images -q docker.io/$(DOCKER_ORG)/$(DOCKER_IMAGE):$(DOCKER_TAG)),)
	@echo No Podman image found.
else
	podman rmi docker.io/$(DOCKER_ORG)/$(DOCKER_IMAGE):$(DOCKER_TAG)
endif
	@echo Podman images cleaned.

clean:
	rm -rf build install log

.PHONY: all dep build build-symlink-install run launch view docker-build docker-push podman-build podman-push docker-clean podman-clean clean