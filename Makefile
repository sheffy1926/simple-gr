# Build the global router

# Command for continuous build: (TO AUTOMATE)
# fswatch -0 -r -l 5 -o ./src/* | xargs -0 -n 1 -I {} "make"

BUILD_DIR ?= $(CURDIR)/build

.PHONY : build
build:
	@echo "🟡 Updating CMake build files..."
	@mkdir -p ${BUILD_DIR}
	@cd ${BUILD_DIR}; cmake ..
	@echo "🟡 Building project..."
	@cmake --build ${BUILD_DIR} 
	@date +"🟢 %T - Build finished"
	@echo

.PHONY : clean
clean:
	@echo "🟧 Cleaning build files in ${BUILD_DIR}..."
	@rm -r ${BUILD_DIR}

.PHONY : bundle
bundle:
	cd ..; tar  --exclude='./.git/' --exclude="./.cache/" --exclude="./goldenRef" \
		--exclude="./benchmarks" --exclude="./build/" --exclude="./cmake" \
		--exclude=".gitignore" \
		-cvzf gr.tar.gz -C ${CURDIR} .
