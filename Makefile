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

# Override to run a specific command in the container via make shell
CMD ?=

### DERIVED VARIABLES
#
# Variables which are not intended to be overridden on the command line.

# Name of image to build.
PROJECT_IMAGE := $(WHOAMI)/$(PROJECT):$(TAG)

# Run a command in the image as ros or root.
DOCKER_RUN_COMMON := $(DOCKER) run -it --privileged
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
SSH := ssh -o StrictHostKeyChecking=no -i config/ros_user_ssh_key -l ros

# Name of the "delete my work" target
REMOVE_SSH_TARGET := delete_all_my_work

.PHONY: default
default: image

.PHONY: image
image:
	$(DOCKER) build -t $(PROJECT_IMAGE) src

# We have to do this curious "double rule" here because the $(shell ...)
# functions are evaluated at the *start* of the rule no matter where within the
# rule they occur.
.PHONY: ssh $(REMOVE_SSH_TARGET)
.PHONY: _existing_ssh_container _launch_ssh_container_if_necessary _ssh_container_valid
_launch_ssh_container_if_necessary: image
	@if [ -z "$(SSH_CID)" ] ; then \
		$(DOCKER_RUN_COMMON) --name=$(SSH_NAME) -d -p 22 $(DOCKER_RUN_ROOT) \
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

$(REMOVE_SSH_TARGET): _existing_ssh_container
	$(DOCKER) stop $(SSH_NAME)
	$(DOCKER) rm $(SSH_NAME)

# Launch a login shell in the image.
.PHONY: shell
shell: ssh
	$(SSH) -X $(SSH_IP) $(CMD)

# Launch a GUI session.
.PHONY: gui
gui: ssh
	$(SSH) -X $(SSH_IP) xinit /usr/bin/lxsession -e LXDE -s Lubuntu -- \
		/usr/bin/Xephyr -screen $(GUI_SCREEN)

