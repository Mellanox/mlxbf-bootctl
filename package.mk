# Include this file from the main Makefile, and define PROJECT_NAME
# before the include statement.

SPEC_NAME:=$(PROJECT_NAME).spec
# Obtain version from RPM spec file
VERSION:=$(shell grep '^Version:' $(SPEC_NAME) | cut -d' ' -f2)
TAR_BASE:=$(PROJECT_NAME)-$(VERSION)
TAR_NAME:=$(TAR_BASE).tar.gz
GIT_FILES:=$(shell git ls-files -co --exclude-standard)

git_dir_pack/$(TAR_NAME): $(GIT_FILES)
	rm -rf git_dir_pack
	mkdir -p git_dir_pack/$(TAR_BASE)
	rsync --relative $(GIT_FILES) git_dir_pack/$(TAR_BASE)
	(cd git_dir_pack; tar -zcvf ../$@ $(TAR_BASE))

.PHONY: tarball
tarball: git_dir_pack/$(TAR_NAME)
	@echo Tarball can be found at $<

.PHONY: pkgclean
pkgclean: srpmclean
	rm -rf git_dir_pack

# RPM-SPECIFIC
RPMBUILD_LOC:=$(shell which rpmbuild)
ifneq ($(RPMBUILD_LOC),)

# Draw pkg name from spec file
RPM_PROJECT_NAME:=$(shell \
  rpmspec -q --qf "%{name}\n" $(SPEC_NAME) | \
  head -n1)
RPM_BASE:=$(shell \
  rpmspec --query --qf "%{name}-%{version}-%{release}\n" $(SPEC_NAME) | \
  head -n1)
SRPM_NAME:=$(RPM_BASE).src.rpm

srpmclean:
	rm -rf RPMBUILD
	rm -f $(RPM_PROJECT_NAME)*.src.rpm

srpm: $(SRPM_NAME)

RPMBUILD/SOURCES/$(TAR_NAME): git_dir_pack/$(TAR_NAME)
	mkdir -p RPMBUILD/SOURCES
	cp $< $@

$(SRPM_NAME): RPMBUILD/SOURCES/$(TAR_NAME) $(SPEC_NAME)
	rpmbuild -bs --define "_topdir $(shell pwd)/RPMBUILD" $(SPEC_NAME)
	cp RPMBUILD/SRPMS/$(SRPM_NAME) ./

else
srpm:
	@echo "Cannot build SRPM, rpmbuild tools not installed!"
	exit 1

srpmclean:

endif

.PHONY: srpmclean srpm
