############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

find_library(LIBCOMMKIT commkit NO_CMAKE_FIND_ROOT_PATH)
find_path(COMMKIT_INC commkit/commkit.h NO_CMAKE_FIND_ROOT_PATH)
find_library(LIBCAPNP capnp NO_CMAKE_FIND_ROOT_PATH)

if (NOT LIBCOMMKIT)
	message(FATAL_ERROR "Commkit not found. Set the CMAKE_PREFIX_PATH environment variable to point to it.")
endif()

get_filename_component(LIBDIR ${LIBCOMMKIT} DIRECTORY)

if(NOT APPLE)
	set(COMMKIT_RPATH "-Wl,-rpath-link,${LIBDIR}")
endif()

include("./schema/schema.cmake")

px4_add_module(
	MODULE modules__rtps
	MAIN rtps
	STACK_MAIN 1200
	COMPILE_FLAGS
		${MODULE_CFLAGS}
		-I ${COMMKIT_INC}
		-I ${SCHEMA_INCLUDE}
	SRCS
		rtps.cpp
		${SCHEMA_CPP}
	DEPENDS
		platforms__common
	LIBRARIES
		${LIBCOMMKIT}
		${LIBCAPNP}
		${COMMKIT_RPATH}
	)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
