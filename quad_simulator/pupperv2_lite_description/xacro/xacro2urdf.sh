#!/bin/sh

# Creates the xacro and sdf file
# TODO: Update shell script s.t. it automatically puts into urdf and sdf directory

xacroFileName="pupperv2_description.urdf.xacro"
urdfFileName="pupperv2_description.urdf"
sdfFileName="pupperv2_description.sdf"


# Removing files
echo "Removing sdf file: $sdfFileName"
rm -rf $sdfFileName
echo "Removing urdf file $urdfFileName" 
rm -rf $urdfFileName

echo "------------------------------------------------------"
# Convert xacro to urdf
echo "Generating urdf file from xacro file: $xacroFileName"
rosrun xacro xacro $xacroFileName > $urdfFileName
echo "Generating sdf file from urdf file: $urdfFileName"
gz sdf -p $urdfFileName > $sdfFileName