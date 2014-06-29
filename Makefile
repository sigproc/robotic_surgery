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
GUI_SCREEN ?= 1440x900

VNC_VIEWER ?= vncviewer

# Any other options to pass to vncviewer
VNC_VIEWER_OPTS ?=

# Override to run a specific command in the container via make shell
CMD ?=

# Override to run a specific command in the container via make gui
GUI_CMD ?= bash

# Override to pass additional arguments to catkin_make via make build
CM_ARGS ?=

# ROS package and launch file and additional parameters to use with roslaunch (make launch)
PKG ?= surgery_launch
LAUNCH ?= surgery.launch
PARAMS ?=

### DERIVED VARIABLES
#
# Variables which are not intended to be overridden on the command line.

# Name of image to build.
PROJECT_IMAGE := $(WHOAMI)/$(PROJECT):$(TAG)

# Docker run options:
#  -it			allocate a pseudo TTY and keep stdin open on interactive
#			sessions (mosly useful for debugging)
#  --privileged		allow container to access hardware
#  -P			open all network ports on the container
#  -v /dev/dri:...	allow X server within container to talk to graphics card
DOCKER_RUN_COMMON := $(DOCKER) run -it --privileged -P -v /dev/dri:/dev/dri

# Run a command in the image as ros or root.
DOCKER_RUN_ROS := -u ros -w /home/ros/workspace -e HOME=/home/ros "$(PROJECT_IMAGE)"
DOCKER_RUN_ROOT := -u root "$(PROJECT_IMAGE)"

# Name of the SSH container and its associated container id, start image and IP address
SSH_NAME := $(WHOAMI)_$(PROJECT)_ssh

SSH_CID_CMD := $(DOCKER) inspect -f '{{ .Id }}' $(SSH_NAME) 2>/dev/null
SSH_IMAGE_ID_CMD := $(DOCKER) inspect -f '{{ .Image }}' $(SSH_NAME) 2>/dev/null
SSH_IP_CMD := $(DOCKER) inspect -f '{{ .NetworkSettings.IPAddress }}' $(SSH_NAME) 2>/dev/null

SSH_CID := $(shell $(SSH_CID_CMD))
SSH_IP := $(shell $(SSH_IP_CMD))
SSH_IMAGE_ID := $(shell $(SSH_IMAGE_ID_CMD))

# The id for the image we have built.
IMAGE_ID_CMD := $(DOCKER) inspect -f '{{ .Id }}' $(PROJECT_IMAGE) 2>/dev/null

# Launch a SSH session into the container
SSH_OPTS := -o StrictHostKeyChecking=no -i config/ros_user_ssh_key
SSH := ssh $(SSH_OPTS) -l ros
SCP := scp $(SSH_OPTS)

# Name of the "delete my work" target
REMOVE_SSH_TARGET := delete_all_my_work

.PHONY: default
default: build

.PHONY: image
image:
	$(DOCKER) build -t $(PROJECT_IMAGE) src

# We have to do this curious "double rule" here because the $(shell ...)
# functions are evaluated at the *start* of the rule no matter where within the
# rule they occur.
.PHONY: ssh $(REMOVE_SSH_TARGET)
.PHONY: _existing_ssh_container _launch_ssh_container_if_necessary _ssh_container_valid

# If a ssh container is not running, launch one. We give it the fixed name
# $(SSH_NAME) and run it in detached mode.
_launch_ssh_container_if_necessary: image
	@if [ -z "$(SSH_CID)" ] ; then \
		$(DOCKER_RUN_COMMON) --name=$(SSH_NAME) -d $(DOCKER_RUN_ROOT) \
			/usr/sbin/sshd -D ; \
	fi

_existing_ssh_container: _launch_ssh_container_if_necessary
	$(eval SSH_CID := $(shell $(SSH_CID_CMD)))
	$(eval SSH_IP := $(shell $(SSH_IP_CMD)))
	$(eval SSH_IMAGE_ID := $(shell $(SSH_IMAGE_ID_CMD)))

_ssh_container_valid: image _existing_ssh_container
	$(eval IMAGE_ID := $(shell $(IMAGE_ID_CMD)))
	@if [ "$(SSH_IMAGE_ID)" != "$(IMAGE_ID)" ] ; then \
		echo "The current SSH container is for an old version of the build image." ; \
		echo "Remove it with\n" ; \
		echo "    $(MAKE) $(REMOVE_SSH_TARGET)\n" ; \
		echo "and try again." ; \
		echo "**IMPORTANT** THIS WILL DELETE ANY LOCAL CHANGES IN THE CONTAINER!" ; \
		false ; \
	fi

ssh: _ssh_container_valid
	scripts/wait_port.sh $(SSH_IP) 22
	@echo "SSH started. Log in with\n"
	@echo "    $(SSH) $(SSH_IP)"

$(REMOVE_SSH_TARGET):
	if [ -n "$(SSH_CID)" ]; then \
		$(DOCKER) stop $(SSH_NAME) ; \
		$(DOCKER) rm $(SSH_NAME) ; \
	fi

# Launch a login shell in the image.
.PHONY: shell
shell: ssh
	$(SSH) -X $(SSH_IP) "$(CMD)"

# Launch a GUI session. This does some horrible magic in order to launch a SSH
# session into the container, send it to the background, launch vncviewer and
# then, after vncviewer exits, kill the original SSH session.
.PHONY: gui
gui: GUI_CMD_TMP := $(shell mktemp -t gui-cmd-robotic-surgery.XXXXXX)
gui: ssh
	echo "$(GUI_CMD)" > "$(GUI_CMD_TMP)"
	$(SCP) $(GUI_CMD_TMP) ros@$(SSH_IP):/home/ros/gui-command.sh
	rm -f $(GUI_CMD_TMP)
	$(SSH) $(SSH_IP) x11vnc -forever -xdummy \
			-env FD_PROG=\"/usr/bin/lxsession -e LXDE -s Lubuntu\" \
			-env FD_GEOM=\"${GUI_SCREEN}\" & \
		scripts/wait_port.sh $(SSH_IP) 5900 && \
		$(VNC_VIEWER) $(VNC_VIEWER_OPTS) $(SSH_IP) ; \
		$(SSH) $(SSH_IP) killall lxsession

# Run a roslaunch file
.PHONY: launch gui_launch
launch: ssh
	$(SSH) -X $(SSH_IP) 'roslaunch $(PKG) $(LAUNCH) $(PARAMS)'

gui_launch: GUI_CMD := roslaunch $(PKG) $(LAUNCH) $(PARAMS)
gui_launch: gui

# Build the actual software
.PHONY: build
build: ssh
	$(SSH) $(SSH_IP) 'cd ~/workspace && catkin_make $(CM_ARGS)'

# Run the tests
.PHONY: test
test: ssh
	$(SSH) $(SSH_IP) 'cd ~/workspace && catkin_make run_tests'
