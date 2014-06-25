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

ssh: build
	ssh_cid=`$(DOCKER_RUN_COMMON) -d -p 22 $(DOCKER_RUN_ROOT) /usr/sbin/sshd -D` ; \
	echo "SSH server launched on `$(DOCKER) inspect -f '{{ .NetworkSettings.IPAddress }}' $$ssh_cid`"

# Launch a login shell in the image.
shell: build
	$(DOCKER_RUN_COMMON) -P --rm $(DOCKER_RUN_ROS) bash -l

# Launch a login shell in the image as the root user.
root_shell: build
	$(DOCKER_RUN_COMMON) -P --rm $(DOCKER_RUN_ROOT) bash -l

# Launch a GUI session.
gui: build
	gui_cid=`$(DOCKER_RUN_COMMON) -p 5900 -d $(DOCKER_RUN_ROS) \
		src/robotic_surgery/scripts/launch_gui.sh $(GUI_SCREEN) lxterminal` ; \
	echo "Spawned GUI container $$gui_cid" ; \
	gui_ip=`$(DOCKER) inspect -f '{{ .NetworkSettings.IPAddress }}' $$gui_cid` ; \
	echo "IP address is $$gui_ip" ; \
	scripts/wait_port.sh $$gui_ip 5900; $(VNC_VIEWER) $$gui_ip:0 ; \
	$(DOCKER) logs $$gui_cid ; \
	echo "Killing $$gui_cid..." ; \
	$(DOCKER) kill $$gui_cid ; \
	$(DOCKER) rm $$gui_cid ;
