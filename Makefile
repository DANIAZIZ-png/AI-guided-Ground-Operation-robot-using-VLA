# VLA ground robot -- one-command entry points.
#
#   make setup     tool venv, model weights (verified), container images
#   make sim       simulation stack
#   make robot     hardware stack
#   make test      unit tests + the refactor parity gate
#   make smoke     headless simulation smoke test
#   make stop      everything down
#
# `make help` lists every target.
#
# WORKS WITH PODMAN OR DOCKER
#   ENGINE is detected, Podman first because that is what this project used.
#   Override with:  make sim ENGINE=docker

SHELL := /bin/bash
.DEFAULT_GOAL := help

VLA_ROOT  := $(shell cd $(dir $(lastword $(MAKEFILE_LIST))) && pwd)
VENV      := $(VLA_ROOT)/.venv-tools
COMPOSE_BIN := $(VENV)/bin/podman-compose

ENGINE ?= $(shell command -v podman >/dev/null 2>&1 && echo podman || echo docker)

# Compose provider. Neither podman-compose nor docker-compose ships with
# Podman, so `make setup` installs podman-compose into a repo-local venv --
# no sudo, and nothing outside the repository is touched.
ifeq ($(ENGINE),docker)
  COMPOSE := docker compose
else
  COMPOSE := $(COMPOSE_BIN)
endif

ROS_IMAGE        := vla-ros:1.0
PERCEPTION_IMAGE := vla-perception:1.0

# Tests need rclpy, which lives in the ROS container. On this machine that is
# the distrobox container; in a clean checkout it is the built image.
# PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 is required: the container's pytest is 6.2.5
# and a user-site anyio plugin wants a newer _pytest.scope, which otherwise
# aborts collection before any test runs.
PYTEST := PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest
ROS_SETUP := source /opt/ros/humble/setup.bash

# Run a command in the ROS environment: the distrobox container if present
# (the development machine), otherwise the built image.
define in_ros
	@if command -v distrobox >/dev/null 2>&1 && distrobox list 2>/dev/null | grep -q ubuntu22-gpu; then \
		distrobox enter ubuntu22-gpu -- bash -c '$(ROS_SETUP); cd $(VLA_ROOT); $(1)'; \
	else \
		$(ENGINE) run --rm -v $(VLA_ROOT):/opt/vla -w /opt/vla $(ROS_IMAGE) bash -c '$(1)'; \
	fi
endef

.PHONY: help
help:
	@echo "VLA ground robot"
	@echo
	@echo "  make setup       tool venv, verified model weights, container images"
	@echo "  make models      download/verify the weights only"
	@echo "  make images      build both container images only"
	@echo "  make sim         start the simulation stack"
	@echo "  make robot       start the hardware stack (robot must be docked and on)"
	@echo "  make stop        stop everything from both profiles"
	@echo
	@echo "  make test        unit tests and the refactor parity gate"
	@echo "  make parity      the parity gate alone"
	@echo "  make lint        ruff"
	@echo "  make smoke       headless simulation smoke test"
	@echo "  make bringup     build the vla_bringup ROS 2 package"
	@echo
	@echo "  engine : $(ENGINE)"
	@echo "  compose: $(COMPOSE)"
	@echo "  root   : $(VLA_ROOT)"

# ---------------------------------------------------------------------------
# setup
# ---------------------------------------------------------------------------
.PHONY: setup
setup: $(COMPOSE_BIN) models images
	@echo
	@echo "Setup complete. Next:  make sim"

$(COMPOSE_BIN):
	@echo "==> tool venv (podman-compose), repo-local, no sudo"
	python3 -m venv $(VENV)
	$(VENV)/bin/pip install --quiet --upgrade pip
	$(VENV)/bin/pip install --quiet podman-compose
	@$(COMPOSE_BIN) --version

.PHONY: tools
tools: $(COMPOSE_BIN)

.PHONY: models
models:
	@echo "==> model weights (every sha256 checked against env/MANIFEST.md)"
	@./scripts/download_models.sh

.PHONY: verify-models
verify-models:
	@./scripts/download_models.sh --verify

.PHONY: images
images: image-ros image-perception

.PHONY: image-ros
image-ros:
	@echo "==> $(ROS_IMAGE) (ROS pinned to the 2026-05-14 snapshot; see env/ROS_SNAPSHOT.md)"
	$(ENGINE) build -f docker/ros.Dockerfile -t $(ROS_IMAGE) .

.PHONY: image-perception
image-perception:
	@echo "==> $(PERCEPTION_IMAGE) (Python 3.12, pins from env/pip-freeze_yolo-env.txt)"
	$(ENGINE) build -f docker/perception.Dockerfile -t $(PERCEPTION_IMAGE) .

# ---------------------------------------------------------------------------
# run
# ---------------------------------------------------------------------------
.PHONY: sim
sim: $(COMPOSE_BIN)
	@echo "==> simulation stack"
	$(COMPOSE) --profile sim up

.PHONY: robot
robot: $(COMPOSE_BIN)
	@echo "==> hardware stack."
	@echo "    The robot must be powered, docked and on the hotspot."
	@echo "    Nav2 and SLAM come from vla_bringup, which carries the"
	@echo "    /parameter_events and /rosout remaps. Do not substitute the stock"
	@echo "    turtlebot4_navigation launch files: they flood the robot's Wi-Fi."
	$(COMPOSE) --profile robot up

.PHONY: stop
stop:
	@echo "==> stopping both profiles"
	-@$(COMPOSE) --profile sim   down --remove-orphans 2>/dev/null
	-@$(COMPOSE) --profile robot down --remove-orphans 2>/dev/null
	@echo "    done. This does NOT touch the distrobox containers or anything"
	@echo "    started by scripts/vla_demo.sh -- use scripts/vla_demo_stop.sh for those."

# ---------------------------------------------------------------------------
# checks
# ---------------------------------------------------------------------------
.PHONY: test
test:
	@echo "==> unit tests and parity gate"
	$(call in_ros,$(PYTEST) tests/ -q)

.PHONY: parity
parity:
	@echo "==> refactor parity gate"
	$(call in_ros,python3 tests/snapshot_agent_api.py vla_agent_v28 --compare tests/baseline_agent_api.json)

.PHONY: test-noros
test-noros:
	@echo "==> only the tests that need no ROS (what CI runs)"
	$(call in_ros,$(PYTEST) tests/test_parity_gate.py tests/test_gui_config.py tests/test_bringup_package.py -q)

.PHONY: lint
lint:
	@echo "==> ruff"
	$(call in_ros,python3 -m ruff check src tests tools scripts vla_bringup || true)

.PHONY: smoke
smoke:
	@echo "==> headless simulation smoke test"
	@./tests/smoke_sim.sh

.PHONY: bringup
bringup:
	@echo "==> colcon build vla_bringup"
	$(call in_ros,mkdir -p /tmp/vla_ws/src && ln -sfn $(VLA_ROOT)/vla_bringup /tmp/vla_ws/src/vla_bringup && cd /tmp/vla_ws && colcon build --packages-select vla_bringup)

.PHONY: clean
clean:
	@echo "==> removing build artefacts (not the images, not the models)"
	find . -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null || true
	find . -name '*.pyc' -delete 2>/dev/null || true
	rm -rf /tmp/vla_ws
