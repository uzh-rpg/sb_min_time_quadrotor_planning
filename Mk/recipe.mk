uniq = $(if $1,$(firstword $1) $(call uniq,$(filter-out $(firstword $1),$1)))

CXXFLAGS:=$(call uniq,$(CXXFLAGS))

clean_make: clean $(TARGET)

bin: $(TARGET)

$(OBJS): %.o: src/%.cpp
	echo "making $<"
	$(CXX) -c $< $(CXXFLAGS) $(CPPFLAGS) -o $(OBJ_DIR)/$@

$(TARGET): create_directories  obj_subdirs $(OBJS) 
	$(CXX) $(CXXFLAGS) -o $@ $(OBJ_DIR)/*.o $(LDFLAGS)


all_subdirs:    
	echo $(OPSYS)
	@for i in $(SUBDIRS) ;\
	do \
	echo "making" all "in $(CURRENT_DIR)/$$i..."; \
	$(MAKE) -C $$i all || exit 1 ;\
	done

obj_subdirs: all_subdirs
	echo "Copy objs"
	@for i in $(SUBDIRS) ;\
	do \
	$(MAKE) --ignore-errors -C $$i obj ;\
	echo "coping all in $(CURRENT_DIR)/$$i..."; \
	cp $$i/*.o $(OBJ_DIR)/; \
	done

create_directories:
	echo "create dircetory $(OBJ_DIR)"
	mkdir -p $(OBJ_DIR)

dependencies: cnpy_lib flann_lib

flann_lib:
	echo "making flann"
	cd ./lib/flann; mkdir -p build; cd build; cmake .. -DCMAKE_INSTALL_PREFIX=../install ; make -j4; make install

cnpy_lib:
	echo "making cnpy"
	cd ./lib/cnpy; mkdir -p build; cd build; cmake .. -DCMAKE_INSTALL_PREFIX=../install ; make -j4; make install

clean_dependencies: clean_flann_lib clean_cnpy_lib

clean_cnpy_lib:
	echo "cleaning cnpy"
	cd ./lib/cnpy; rm -rf build; rm -rf install

clean_flann_lib:
	echo "cleaning flann"
	cd ./lib/flann; rm -rf build; rm -rf install

clean:
	$(RM) $(TARGET)
	$(RM) $(OBJ_DIR)/*
	@for i in $(SUBDIRS) ;\
        do \
        echo "cleaning" all "in $(CURRENT_DIR)/$$i..."; \
        $(MAKE) -C $$i clean; \
        done

clean_all: clean clean_dependencies

