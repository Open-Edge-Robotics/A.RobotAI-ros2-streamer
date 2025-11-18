# Makefile for ros-streamer project

# ==============================================================================
# Variables
# ==============================================================================

# Shell
SHELL := /bin/bash

# Project settings
PROJECT_NAME := ros-streamer
PACKAGE_NAME := ros_stream_server
NODE_NAME := stream_server
PYTHON := python3

# ROS settings
ROS_DISTRO ?= humble
ROS2_SETUP := /opt/ros/$(ROS_DISTRO)/setup.bash
WS_SETUP := $(shell pwd)/install/setup.bash

# Docker settings
DOCKER_IMAGE_NAME := $(PROJECT_NAME)
DOCKER_TAG := latest
DOCKER_FULL_IMAGE_NAME := $(DOCKER_IMAGE_NAME):$(DOCKER_TAG)

# Kubernetes settings
KUBE_DEPLOYMENT_FILE := src/ros_stream_server/kube/ros-streamer-deployment.yaml


# ==============================================================================
# Targets
# ==============================================================================

.PHONY: all help deps build build-symlink run launch test view clean docker-build docker-push docker-all

all: build

help:
	@echo "Usage: make [target]"
	@echo ""
	@echo "Available targets:"
	@echo "  help                  - Show this help message"
	@echo ""
	@echo "  --- Development ---"
	@echo "  deps                  - Install Python and ROS dependencies"
	@echo "  build                 - Build the ROS package"
	@echo "  build-symlink         - Build with symlink install for faster development"
	@echo "  run                   - Run the stream_server node"
	@echo "  launch                - Launch the stream_server using its launch file"
	@echo "  test                  - Run automated tests for the package"
	@echo "  view                  - Run the separate viewer application"
	@echo "  clean                 - Remove build, install, and log artifacts"
	@echo ""
	@echo "  --- Docker ---"
	@echo "  docker-build          - Build the Docker image for the project"
	@echo "  docker-push           - Push the Docker image to a registry"
	@echo "  docker-all            - Build and push the Docker image"


# --- Development Targets ---

deps:
	@echo "--> Installing Python dependencies from requirements.txt..."
	$(PYTHON) -m pip install -r requirements.txt
	@echo "--> Installing ROS dependencies..."
	@. $(ROS2_SETUP) && rosdep install --from-paths src --ignore-src -r -y || echo "rosdep check failed, continuing..."

build: deps
	@echo "--> Building ROS package..."
	. $(ROS2_SETUP) && colcon build

build-symlink: deps
	@echo "--> Building ROS package with symlink install..."
	. $(ROS2_SETUP) && colcon build --symlink-install

run: build
	@echo "--> Running node [$(NODE_NAME)] from package [$(PACKAGE_NAME)]..."
	. $(ROS2_SETUP) && . $(WS_SETUP) && ros2 run $(PACKAGE_NAME) $(NODE_NAME)

launch: build
	@echo "--> Launching [$(NODE_NAME).launch.py]..."
	. $(ROS2_SETUP) && . $(WS_SETUP) && ros2 launch $(PACKAGE_NAME) $(NODE_NAME).launch.py

test: build
	@echo "--> Running tests..."
	. $(ROS2_SETUP) && colcon test
	@echo "--> Test results are available in the 'log' directory."

view:
	@echo "--> Starting viewer application..."
	$(PYTHON) ros_stream_viewer/viewer.py

clean:
	@echo "--> Cleaning up build artifacts..."
	rm -rf build install log


# --- Docker Targets ---

docker-build:
	@echo "--> Building Docker image: $(DOCKER_FULL_IMAGE_NAME)"
	@docker build -t $(DOCKER_FULL_IMAGE_NAME) .
	@echo "--> Updating Kubernetes deployment file with new image tag..."
	@sed -i 's|image: .*|image: $(DOCKER_FULL_IMAGE_NAME)|' $(KUBE_DEPLOYMENT_FILE)
	@echo "--> Build complete. Image: $(DOCKER_FULL_IMAGE_NAME)"

docker-push:
	@echo "--> Pushing Docker image: $(DOCKER_FULL_IMAGE_NAME)"
	@docker push $(DOCKER_FULL_IMAGE_NAME)

docker-all: docker-build docker-push
