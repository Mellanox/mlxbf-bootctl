# Include this file from the main Makefile, and define PROJECT_NAME
# before the include statement.

SPEC_NAME:=$(PROJECT_NAME).spec
# Draw pkg name from spec file
PROJECT_NAME:=$(shell \
  rpmspec -q --qf "%{name}\n" $(SPEC_NAME) | \
  head -n1)
RPM_BASE:=$(shell \
  rpmspec --query --qf "%{name}-%{version}-%{release}\n" $(SPEC_NAME) | \
  head -n1)
SRPM_NAME:=$(RPM_BASE).src.rpm
TAR_BASE:=$(shell \
  rpmspec --query --qf "%{name}-%{version}\n" $(SPEC_NAME) | \
  head -n1)
TAR_NAME:=$(TAR_BASE).tar.gz
GIT_FILES:=$(shell git ls-files -co --exclude-standard)

.PHONY: pkgclean srpm

pkgclean:
	rm -rf RPMBUILD
	rm -rf git_dir_pack
	rm -f $(PROJECT_NAME)*.src.rpm

srpm: $(SRPM_NAME)

RPMBUILD/SOURCES/$(TAR_NAME): $(GIT_FILES)
	mkdir -p RPMBUILD/SOURCES
	rm -rf git_dir_pack
	mkdir -p git_dir_pack/$(TAR_BASE)
	rsync --relative $(GIT_FILES) git_dir_pack/$(TAR_BASE)
	(cd git_dir_pack; tar -zcvf ../$@ $(TAR_BASE))

$(SRPM_NAME): RPMBUILD/SOURCES/$(TAR_NAME) $(SPEC_NAME)
	rpmbuild -bs --define "_topdir $(shell pwd)/RPMBUILD" $(SPEC_NAME)
	cp RPMBUILD/SRPMS/$(SRPM_NAME) ./
