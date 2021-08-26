#!/bin/bash
echo "Executing the configure script..."
echo " "
kvsmake -g tmp_exec
echo "Makefile.kvs is created."
echo " "
echo "Edit INSTALL_DIR in Makefile if necessary:"
echo "  INSTALL_DIR=\$(HOME)/local/bin"