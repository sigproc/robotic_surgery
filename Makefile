.PHONY: default help build shell root_shell gui _gui

.DEFAULT: default

### CUSTOMISABLE VARIABLES
#
# These variables have default values which may be overridden on the command line.

# Username used to prefix Docker images with.
WHOAMI ?= ${USER}

# Docker binary
DOCKER ?= docker

# What is a suitable name for the current git branch? Use it as the Docker tag.
# If this is not a git repo, or there is no HEAD ref, use "latest" as a
# fallback.
TAG ?= $(shell git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "latest")

# The name of this project
PROJECT ?= robotic_surgery

# The size of the GUI screen
GUI_SCREEN ?= 1440x960

# The VNC viewer command. Should accept a host:display style option
VNC_VIEWER ?= vncviewer

# Arguments to pass to roslaunch
LAUNCH ?=

### DERIVED VARIABLES
#
# Variables which are not intended to be overridden on the command line.

# Name of image to build.
PROJECT_IMAGE := $(WHOAMI)/$(PROJECT):$(TAG)

# Run a command in the image as ros or root.
DOCKER_RUN_COMMON := $(DOCKER) run -it --privileged
DOCKER_RUN_ROS := -u ros -w /home/ros/workspace -e HOME=/home/ros "$(PROJECT_IMAGE)"
DOCKER_RUN_ROOT := -u root "$(PROJECT_IMAGE)"

# Locations and values of various temporary variables files
SSH_CID_FILE := .ssh_container_id
SSH_CID = $(shell cat $(SSH_CID_FILE))
SSH_IP = $(shell $(DOCKER) inspect -f '{{ .NetworkSettings.IPAddress }}' $(SSH_CID))

# The id for the image we have built
BUILD_IMAGE_ID = $(shell $(DOCKER) inspect -f '{{ Id }}' $(PROJECT_IMAGE))

# The id for the image which the ssh container is currently running in
SSH_IMAGE_ID = $(shell $(DOCKER) inspect -f '{{ Image }}' $(SSH_CID))

# Launch a SSH session into the container
SSH := ssh -o StrictHostKeyChecking=no -i config/ssh/id_rsa -l ros

# Command used to launch a login shell into the image
LOGIN_RUN_OPTS = $(COMMON_RUN_OPTS) /bin/bash -l

BUILD_RUN_OPTS = $(COMMON_RUN_OPTS) /bin/bash -c \
		'source ~/workspace/devel/setup.bash; catkin_make'

TEST_RUN_OPTS = $(COMMON_RUN_OPTS) /bin/bash -c \
		'source ~/workspace/devel/setup.bash; catkin_make run_tests'

default: build

# Build Docker image from our top-level Dockerfile
build:
	$(DOCKER) build -t $(PROJECT_IMAGE) .
	@echo "Docker image built. Id: $(BUILD_IMAGE_ID)"

ssh: $(SSH_IP_FILE)
	@echo Log into container as ros@$(SSH_IP)

$(CONTAINER_DIR): build
	mkdir -p .container

$(SSH_CID_FILE): $(CONTAINER_DIR)
	echo $(BUILD_IMAGE_ID) > $(SSH_IMAGE_ID_FILE)
	$(DOCKER_RUN_COMMON) -d -p 22 $(DOCKER_RUN_ROOT) /usr/sbin/sshd -D > $(SSH_CID_FILE)

$(SSH_IP_FILE): $(SSH_CID_FILE)
	$(DOCKER) inspect -f '{{ .NetworkSettings.IPAddress }}' $(SSH_CID) > $(SSH_IP_FILE)

# Launch a login shell in the image.
shell: build
	$(DOCKER_RUN_COMMON) -P --rm $(DOCKER_RUN_ROS) bash -l

# Launch a login shell in the image as the root user.
root_shell: build
	$(DOCKER_RUN_COMMON) -P --rm $(DOCKER_RUN_ROOT) bash -l

# Launch a GUI session.
gui: ssh
	scripts/wait_port.sh $(SSH_IP) 22
	$(SSH) $(SSH_IP) -X xinit /usr/bin/lxsession -e LXDE -s Lubuntu -- \
		/usr/bin/Xephyr -screen $(GUI_SCREEN)
