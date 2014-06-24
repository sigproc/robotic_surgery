### CUSTOMISABLE VARIABLES

# Username used to prefix Docker images with.
WHOAMI ?= ${USER}

# Location of Docker binary
DOCKER ?= docker

# What is a suitable name for the current git branch? Use it as the Docker tag.
# If this is not a git repo, or there is no HEAD ref, use "latest" as a
# fallback.
TAG ?= $(shell git name-rev HEAD --name-only 2>/dev/null || echo "latest")

### DERIVED VARIABLES

# Name of image to build
DEV_IMAGE_NAME = $(WHOAMI)/robotic-surgery:$(TAG)

# Command used to launch a login shell into the image
LOGIN_CMD = "$(DOCKER)" run --rm -ti -u ros -w /home/ros -e HOME=/home/ros \
	"$(DEV_IMAGE_NAME)" /bin/bash -l

all: build

# Build a Docker image for this repo
build:
	"$(DOCKER)" build -t "$(DEV_IMAGE_NAME)" .

# Launch a shell in the Docker image *after* building
login: build
	$(LOGIN_CMD)

# Launch a shell in the Docker image *without* building. Usually you don't want
# to do this.
no-build-login:
	$(LOGIN_CMD)

.PHONY: all build login

.DEFAULT: all
