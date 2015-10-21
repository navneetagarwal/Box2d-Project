.SUFFIXES: .cpp .hpp

# Programs
SHELL 	= bash
CC     	= g++
LD	= ld
RM 	= rm
ECHO	= /bin/echo
CAT	= cat
PRINTF	= printf
SED	= sed
DOXYGEN = doxygen
######################################
# Project Name (generate executable with this name)
TARGET = cs251_base_exe_02
TARGET1 = dirs
TARGET2 = b2dsetup
TARGET3 = setup
TARGET4 = exe

# Project Paths
PROJECT_ROOT=./
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
PRJSRC = $(PROJECT_ROOT)/external/src
OBJDIR = $(PROJECT_ROOT)/obj
BINDIR = $(PROJECT_ROOT)/bin
DOCDIR = $(PROJECT_ROOT)/doc
OBJMYDIR = $(PROJECT_ROOT)/obj
BINMYDIR = $(PROJECT_ROOT)/bin

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGSN =-g -O3 -Wall -fno-strict-aliasing
CPPFLAGSN+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
CPPFLAGS =-g -Wall -fno-strict-aliasing
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib

######################################

NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJMYDIR)/%.o)


.PHONY: all setup codeDoc clean distclean

all: setup 
	


dirs: 
	@mkdir -p bin
	@mkdir -p obj
setup: dirs b2dsetup
b2dsetup:
	@if [ test -e ./external/include/Box2D && test -e ./external/lib/Box2D ]; \
	then $(ECHO) " "; \
	else $(ECHO) "Installing Box2D"; \
	cd $(PRJSRC); \
	tar -zxf Box2D.tgz; \
	cd Box2D; \
	mkdir build251; \
	cd build251; \
	cmake ../; \
	make; \
	make install; \
	fi;
release:$(BINMYDIR)/$(TARGET)

$(BINMYDIR)/$(TARGET): $(OBJS)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $@)"
	@$(CC) -o $@ $(LDFLAGS) -pg $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err

-include -include $(OBJS:.o=.d)

$(OBJS): $(OBJMYDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) -pg $(CPPFLAGS) -pg -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err

codeDoc:
	@$(ECHO) -n "Generating Doxygen Documentation of Group 02 ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@$(ECHO) "Done"

report:
	@cd ./doc; \
	pdflatex report.tex; \
	bibtex report.aux; \
	pdflatex report.tex; \
	pdflatex report.tex; \
	pdflatex Beamer.tex; \

profile:
	@gprof ./bin/cs251_base_exe_02 gmon.out>project.txt;
	@python3 gprof2dot.py project.txt>project.dot
	@dot -Tpng project.dot -o project.png

clean:
	@$(ECHO) -n "Cleaning up..."
	@$(RM) -rf $(OBJMYDIR)/*
	@$(RM) -rf $(BINMYDIR)/*
	@$(RM) -f ./doc/report.aux
	@$(RM) -f ./doc/report.bbl
	@$(RM) -f ./doc/report.blg
	@$(RM) -f ./doc/report.log
	@$(RM) -f ./doc/report.out
	@$(RM) -f ./doc/report.run.xml
	@$(RM) -f ./doc/report-blx.bib
	@$(RM) -f ./doc/report.pdf
	@$(RM) -f ./doc/report.toc
	@$(RM) -f ./doc/Beamer.log
	@$(RM) -f ./doc/Beamer.aux
	@$(RM) -f ./doc/Beamer.nav
	@$(RM) -f ./doc/Beamer.out
	@$(RM) -f ./doc/Beamer.snm
	@$(RM) -f ./doc/Beamer.toc
	@$(RM) -f ./doc/texput.log
	@$(RM) -f ./project.dot
	@$(RM) -f ./project.txt
	@$(RM) -f ./project.png
	@$(RM) -f ./gmon.out
	@$(RM) -f ./doc/Beamer.pdf
	@$(ECHO) "Done"

distclean: clean
	@$(RM) -rf $(BINMYDIR) $(OBJMYDIR)
	@$(RM) -rf $(EXTERNAL_ROOT)/$(SRCDIR)/Box2D
