UV_CACHE_DIR ?= .uv-cache
XCODE_DEVELOPER_DIR ?= /Applications/Xcode.app/Contents/Developer
IOS_PROJECT ?= apps/ios/RobotTeleop/RobotTeleop.xcodeproj
IOS_SCHEME ?= RobotTeleop
IOS_DESTINATION ?= platform=iOS Simulator,name=iPhone 17 Pro Max,OS=26.5
IOS_DERIVED_DATA ?= build/ios-derived
IOS_NETWORK_TEST_TARGET ?= RobotTeleopUITests/RobotTeleopUITests/testSimulatorUDPProtocolRoundTrip
IOS_XCODEBUILD = DEVELOPER_DIR=$(XCODE_DEVELOPER_DIR) xcodebuild

.PHONY: sync format lint typecheck test check run-mock run-arkit-mock loopback network-test phase3-mock-validation phase3-mock-session phase4-ghost-review phase4-kinematics-check phase5-check arx5-sdk-inspect arx5-state-readiness-dry-run phase5-sdk-model-check phase5-safety-contract phase5-motion-preflight phase5-safe-stop-dry-run phase5-hold-current-dry-run ios-format ios-build ios-test ios-network-test ios-network-test-120

sync:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv sync --group dev

format:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run ruff format .

lint:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run ruff check .
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run ruff format --check .

typecheck:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run mypy

test:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run pytest

check: lint typecheck test

run-mock:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run robot-teleop run --config configs/profiles/mock_mock.yaml --duration 1.0

run-arkit-mock:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run robot-teleop run --config configs/profiles/arkit_mock.yaml --duration 5.0

loopback:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run python tools/udp_loopback.py --count 100

network-test:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run python tools/network_test.py --duration 2.0 --rate-hz 500

phase3-mock-validation:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run python tools/phase3_mock_validation.py --duration 2

phase3-mock-session:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run python tools/phase3_mock_session.py

phase4-ghost-review:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run python tools/phase4_ghost_review.py

phase4-kinematics-check:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run --extra kinematics python tools/phase4_kinematics_check.py

phase5-check:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run --extra arx5 pytest tests/unit/test_arx5_backend.py tests/unit/test_arx5_safety.py tests/unit/test_arx5_sdk_inspect.py tests/unit/test_arx5_state_readiness.py tests/unit/test_phase5_safe_stop.py tests/unit/test_phase5_hold_current.py tests/unit/test_cartesian_ruckig.py tests/unit/test_phase5_sdk_model_check.py tests/unit/test_phase5_safety_contract.py tests/unit/test_ruckig_servo.py tests/integration/test_phase5_motion_preflight.py

arx5-sdk-inspect:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run --extra arx5 python tools/arx5_sdk_inspect.py --model X5

arx5-state-readiness-dry-run:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run --extra arx5 python tools/arx5_state_readiness.py

phase5-sdk-model-check:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run --extra arx5 python tools/phase5_sdk_model_check.py

phase5-safety-contract:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run --extra arx5 python tools/phase5_safety_contract.py

phase5-motion-preflight:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run --extra arx5 python tools/phase5_motion_preflight.py

phase5-safe-stop-dry-run:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run --extra arx5 python tools/phase5_safe_stop.py

phase5-hold-current-dry-run:
	UV_CACHE_DIR=$(UV_CACHE_DIR) uv run --extra arx5 python tools/phase5_hold_current.py

ios-format:
	DEVELOPER_DIR=$(XCODE_DEVELOPER_DIR) xcrun swift-format lint --strict --recursive apps/ios/RobotTeleop

ios-build:
	$(IOS_XCODEBUILD) build -quiet -configuration Release -project $(IOS_PROJECT) -scheme $(IOS_SCHEME) -destination 'generic/platform=iOS Simulator' -derivedDataPath $(IOS_DERIVED_DATA) CODE_SIGNING_ALLOWED=NO
	$(IOS_XCODEBUILD) build -quiet -configuration Release -project $(IOS_PROJECT) -scheme $(IOS_SCHEME) -destination 'generic/platform=iOS' -derivedDataPath $(IOS_DERIVED_DATA) CODE_SIGNING_ALLOWED=NO

ios-test: ios-format
	$(IOS_XCODEBUILD) test -quiet -project $(IOS_PROJECT) -scheme $(IOS_SCHEME) -destination '$(IOS_DESTINATION)' -derivedDataPath $(IOS_DERIVED_DATA) CODE_SIGNING_ALLOWED=NO

ios-network-test: ios-format
	$(IOS_XCODEBUILD) build-for-testing -quiet -project $(IOS_PROJECT) -scheme $(IOS_SCHEME) -destination '$(IOS_DESTINATION)' -derivedDataPath $(IOS_DERIVED_DATA) CODE_SIGNING_ALLOWED=NO
	@mkdir -p build
	@set -eu; \
		UV_CACHE_DIR=$(UV_CACHE_DIR) uv run python tools/pose_monitor.py --bind-host 127.0.0.1 --port 5005 --duration 60 --exit-after-round-trip > build/ios-network-monitor.json & \
		monitor_pid=$$!; \
		trap 'kill "$$monitor_pid" 2>/dev/null || true' EXIT INT TERM; \
		sleep 1; \
		$(IOS_XCODEBUILD) test-without-building -quiet -project $(IOS_PROJECT) -scheme $(IOS_SCHEME) -destination '$(IOS_DESTINATION)' -derivedDataPath $(IOS_DERIVED_DATA) -only-testing:$(IOS_NETWORK_TEST_TARGET) CODE_SIGNING_ALLOWED=NO; \
		wait "$$monitor_pid"; \
		trap - EXIT INT TERM
	@cat build/ios-network-monitor.json

ios-network-test-120:
	$(MAKE) ios-network-test IOS_NETWORK_TEST_TARGET=RobotTeleopUITests/RobotTeleopUITests/testSimulatorUDPProtocolRoundTripAt120Hz
