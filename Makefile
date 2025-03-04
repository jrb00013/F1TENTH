# Define paths to the project directories
SRC_DIR = src
BUILD_DIR = build
BIN_DIR = bin
LIB_DIR = lib
PYTHON_ENV = venv

# Python dependencies for F1TENTH projects
PYTHON_DEPS = requests numpy opencv-python scikit-learn

# C dependencies for F1TENTH projects (for simulation-related files)
C_DEPS = -lm

# Files to compile
C_FILES = $(SRC_DIR)/aeb.c $(SRC_DIR)/slam.c $(SRC_DIR)/rrt.c
C_OBJECTS = $(C_FILES:.c=.o)
C_EXEC = $(BIN_DIR)/f1tenth_simulation

# Python files
PYTHON_FILES = $(SRC_DIR)/bot.py $(SRC_DIR)/follow_gap.py $(SRC_DIR)/sensor_fusion.py $(SRC_DIR)/lidar_processing.py $(SRC_DIR)/slam.py $(SRC_DIR)/pure_pursuit.py $(SRC_DIR)/behavior_planner.py $(SRC_DIR)/ethics_module.py $(SRC_DIR)/object_detection.py $(SRC_DIR)/lane_detection.py

# Default target
all: setup_python_env compile_c_files run_simulation

# Set up Python virtual environment and install dependencies
setup_python_env:
	@echo "Setting up Python virtual environment..."
	@python3 -m venv $(PYTHON_ENV)
	@$(PYTHON_ENV)/bin/pip install -r requirements.txt

# Compile C files
compile_c_files: $(C_OBJECTS)
	@echo "Compiling C files..."
	@$(CC) $(C_OBJECTS) -o $(C_EXEC) $(C_DEPS)

# C file compilation rule
$(SRC_DIR)/%.o: $(SRC_DIR)/%.c
	@$(CC) -c $< -o $@ $(C_DEPS)

# Run the simulation (Python)
run_simulation: $(PYTHON_FILES)
	@echo "Running Python-based simulation..."
	@python3 $(SRC_DIR)/bot.py

# Clean build artifacts
clean:
	@echo "Cleaning build files..."
	@rm -rf $(BUILD_DIR) $(BIN_DIR) $(PYTHON_ENV)

# Test
test:
	@echo "Running tests..."
	@python3 -m unittest discover -s tests

.PHONY: all setup_python_env compile_c_files run_simulation clean test
