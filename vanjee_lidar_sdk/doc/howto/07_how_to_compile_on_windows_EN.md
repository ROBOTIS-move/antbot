# 7 How to Compile vanjee_driver on Windows

## 7.1 Overview

The compilation instructions here are for two parts of vanjee_driver.

The example programs include `demo_online` and `demo_pcap`.

The following steps are completed on a Windows 10 system using VS2019 as the compilation tool.

## 7.2 Compile demo_online

The compilation of the demonstration program is illustrated using `demo_online` as an example. The compilation steps for `demo_pcap` are the same as those for `demo_online`.

## 7.2.1 Install Third-party Libraries

If parsing PCAP files is required, the `libpcap` library needs to be installed, including:

- WpdPack_4_1_2.zip: Header files and library files required during compilation
- WinPcap_4_1_3.exe: Includes runtime libraries

Extract `WpdPack_4_1_2.zip` to the directory C:/Program Files. Run `WinPcap_4_1_3.exe` and install it to the directory C:/Program Files.

Download Eigen3 (https://eigen.tuxfamily.org/index.php?title=Main_Page#Download) Extract `eigen-3.4.0.zip` to the directory C:/Program Files.

## 7.2.2 Create the demo_online Project

Create the `demo_online` project and add the source file `demo_online.cpp`.

## 7.2.3 Configure the demo_online Project

Follow the C++14 standard: Project Properties -> Configuration Properties -> C/C++ -> Language -> C++ Language Standard (ISO C++14 Standard)

`demo_online` depends on the `vanjee_driver` library. Set the header file path for `vanjee_driver` and Eigen. Project Properties -> C/C++ -> General -> Additional Include Directories (../../src) Project Properties -> C/C++ -> General -> Additional Include Directories (C:/Program Files/eigen-3.4.0)

Set the header file path for the `libpcap` library. Project Properties -> Configuration Properties -> VC++ Directories -> Include Directories (C:/Program Files/WpdPack_4_1_2/WpdPack/Include)

Set the library file path for the `libpcap` library. Project Properties -> Configuration Properties -> VC++ Directories -> Library Directories (C:/Program Files/WpdPack_4_1_2/WpdPack/Lib/x64)

Set the dependency library `wpcap.lib` for the `libpcap` library. Also set `ws2_32.lib`, which is the Windows socket library that `vanjee_driver` depends on. Project Properties -> Linker -> Input -> Additional Dependencies (Ws2_32.lib;wpcap.lib)

Set the compilation option `_CRT_SECURE_NO_WARNINGS` to avoid unnecessary compilation errors. Project Properties -> C/C++ -> Preprocessor -> Preprocessor Definitions (_CRT_SECURE_NO_WARNINGS)

If the project prompts for many undefined variables, try setting additional options to add utf-8. Project Properties -> C/C++ -> All Options -> Additional Options (/utf-8)

## 7.2.4 Compile and Run

Compile the `demo_online` project, set it as the startup item, and run it.

## 7.3 Compile vanjee_driver_viewer

Similar to `demo_online`, install the `libpcap` library.

`vanjee_driver_viewer` also depends on the PCL library, which in turn depends on a series of libraries such as Boost, Eigen, etc. Fortunately, PCL provides an installer for MSVC2019, which comes with the libraries it depends on. Here, the installation package used is: PCL-1.11.1-AllInOne-msvc2019-win64.exe

Run it and install it to the directory C:/Program Files. (Add PCL to the system PATH for all users)

Note that when installing the PCL library, it also installs the libraries it depends on. (PCL + 3rd Party Libraries)

The installed component locations are as follows: C:\Program Files\PCL 1.11.1 # PCL itself C:\Program Files\OpenNI2 # OpenNI2 library, which PCL depends on C:\Program Files\PCL 1.11.1\3rdParty # Other libraries that PCL depends on

## 7.3.1 Configure Third-party Libraries

Add the following runtime library paths to the PATH environment variable. C:\Program Files\PCL 1.11.1\bin C:\Program Files\PCL 1.11.1\3rdParty\VTK\bin C:\Program Files\PCL 1.11.1\3rdParty\OpenNI2\Redist

## 7.3.2 Create the vanjee_driver_viewer Project

Create the `vanjee_driver_viewer` project and add the source

## 7.3.3 Configure the vanjee_driver_viewer Project

Similar to `demo_online`:

1. Follow the C++14 standard.
2. Disable SDL checks.
3. Set the header file path for `vanjee_driver`.

Set the header file paths for the PCL library as follows (same as `demo_online`, while also setting the `libpcap` library):

- C:\Program Files\PCL 1.11.1\include\pcl-1.11
- C:\Program Files\PCL 1.11.1\3rdParty\Boost\include\boost-1_74
- C:\Program Files\PCL 1.11.1\3rdParty\Eigen\eigen3
- C:\Program Files\PCL 1.11.1\3rdParty\FLANN\include
- C:\Program Files\PCL 1.11.1\3rdParty\Qhull\include
- C:\Program Files\PCL 1.11.1\3rdParty\VTK\include\vtk-8.2
- C:\Program Files\OpenNI2\Include

Set the library file paths for the PCL library as follows (same as `demo_online`, while also setting the `libpcap` library):

- C:\Program Files\PCL 1.11.1\lib
- C:\Program Files\PCL 1.11.1\3rdParty\Boost\lib
- C:\Program Files\PCL 1.11.1\3rdParty\FLANN\lib
- C:\Program Files\PCL 1.11.1\3rdParty\Qhull\lib
- C:\Program Files\PCL 1.11.1\3rdParty\VTK\lib
- C:\Program Files\OpenNI2\Lib

Set the libraries for PCL and its dependencies, including PCL and VTK (same as `demo_online`, while also setting `wpcap.lib` and `ws2_32.lib`): The library files for PCL are as follows:

pcl_common.lib
pcl_commond.lib
pcl_features.lib
pcl_featuresd.lib
pcl_filters.lib
pcl_filtersd.lib
pcl_io.lib
pcl_iod.lib
pcl_io_ply.lib
pcl_io_plyd.lib
pcl_kdtree.lib
pcl_kdtreed.lib
pcl_keypoints.lib
pcl_keypointsd.lib
pcl_ml.lib
pcl_mld.lib
pcl_octree.lib
pcl_octreed.lib
pcl_outofcore.lib
pcl_outofcored.lib
pcl_people.lib
pcl_peopled.lib
pcl_recognition.lib
pcl_recognitiond.lib
pcl_registration.lib
pcl_registrationd.lib
pcl_sample_consensus.lib
pcl_sample_consensusd.lib
pcl_search.lib
pcl_searchd.lib
pcl_segmentation.lib
pcl_segmentationd.lib
pcl_stereo.lib
pcl_stereod.lib
pcl_surface.lib
pcl_surfaced.lib
pcl_tracking.lib
pcl_trackingd.lib
pcl_visualization.lib
pcl_visualizationd.lib

7.The vtk library is divided into debug/release versions. Here is the release version. The steps here are exemplified by the release version.
vtkChartsCore-8.2.lib
vtkCommonColor-8.2.lib
vtkCommonComputationalGeometry-8.2.lib
vtkCommonCore-8.2.lib
vtkCommonDataModel-8.2.lib
vtkCommonExecutionModel-8.2.lib
vtkCommonMath-8.2.lib
vtkCommonMisc-8.2.lib
vtkCommonSystem-8.2.lib
vtkCommonTransforms-8.2.lib
vtkDICOMParser-8.2.lib
vtkDomainsChemistry-8.2.lib
vtkDomainsChemistryOpenGL2-8.2.lib
vtkdoubleconversion-8.2.lib
vtkexodusII-8.2.lib
vtkexpat-8.2.lib
vtkFiltersAMR-8.2.lib
vtkFiltersCore-8.2.lib
vtkFiltersExtraction-8.2.lib
vtkFiltersFlowPaths-8.2.lib
vtkFiltersGeneral-8.2.lib
vtkFiltersGeneric-8.2.lib
vtkFiltersGeometry-8.2.lib
vtkFiltersHybrid-8.2.lib
vtkFiltersHyperTree-8.2.lib
vtkFiltersImaging-8.2.lib
vtkFiltersModeling-8.2.lib
vtkFiltersParallel-8.2.lib
vtkFiltersParallelImaging-8.2.lib
vtkFiltersPoints-8.2.lib
vtkFiltersProgrammable-8.2.lib
vtkFiltersSelection-8.2.lib
vtkFiltersSMP-8.2.lib
vtkFiltersSources-8.2.lib
vtkFiltersStatistics-8.2.lib
vtkFiltersTexture-8.2.lib
vtkFiltersTopology-8.2.lib
vtkFiltersVerdict-8.2.lib
vtkfreetype-8.2.lib
vtkGeovisCore-8.2.lib
vtkgl2ps-8.2.lib
vtkglew-8.2.lib
vtkGUISupportMFC-8.2.lib
vtkhdf5-8.2.lib
vtkhdf5_hl-8.2.lib
vtkImagingColor-8.2.lib
vtkImagingCore-8.2.lib
vtkImagingFourier-8.2.lib
vtkImagingGeneral-8.2.lib
vtkImagingHybrid-8.2.lib
vtkImagingMath-8.2.lib
vtkImagingMorphological-8.2.lib
vtkImagingSources-8.2.lib
vtkImagingStatistics-8.2.lib
vtkImagingStencil-8.2.lib
vtkInfovisCore-8.2.lib
vtkInfovisLayout-8.2.lib
vtkInteractionImage-8.2.lib
vtkInteractionStyle-8.2.lib
vtkInteractionWidgets-8.2.lib
vtkIOAMR-8.2.lib
vtkIOAsynchronous-8.2.lib
vtkIOCityGML-8.2.lib
vtkIOCore-8.2.lib
vtkIOEnSight-8.2.lib
vtkIOExodus-8.2.lib
vtkIOExport-8.2.lib
vtkIOExportOpenGL2-8.2.lib
vtkIOExportPDF-8.2.lib
vtkIOGeometry-8.2.lib
vtkIOImage-8.2.lib
vtkIOImport-8.2.lib
vtkIOInfovis-8.2.lib
vtkIOLegacy-8.2.lib
vtkIOLSDyna-8.2.lib
vtkIOMINC-8.2.lib
vtkIOMovie-8.2.lib
vtkIONetCDF-8.2.lib
vtkIOParallel-8.2.lib
vtkIOParallelXML-8.2.lib
vtkIOPLY-8.2.lib
vtkIOSegY-8.2.lib
vtkIOSQL-8.2.lib
vtkIOTecplotTable-8.2.lib
vtkIOVeraOut-8.2.lib
vtkIOVideo-8.2.lib
vtkIOXML-8.2.lib
vtkIOXMLParser-8.2.lib
vtkjpeg-8.2.lib
vtkjsoncpp-8.2.lib
vtklibharu-8.2.lib
vtklibxml2-8.2.lib
vtklz4-8.2.lib
vtklzma-8.2.lib
vtkmetaio-8.2.lib
vtkNetCDF-8.2.lib
vtkogg-8.2.lib
vtkParallelCore-8.2.lib
vtkpng-8.2.lib
vtkproj-8.2.lib
vtkpugixml-8.2.lib
vtkRenderingAnnotation-8.2.lib
vtkRenderingContext2D-8.2.lib
vtkRenderingContextOpenGL2-8.2.lib
vtkRenderingCore-8.2.lib
vtkRenderingExternal-8.2.lib
vtkRenderingFreeType-8.2.lib
vtkRenderingGL2PSOpenGL2-8.2.lib
vtkRenderingImage-8.2.lib
vtkRenderingLabel-8.2.lib
vtkRenderingLOD-8.2.lib
vtkRenderingOpenGL2-8.2.lib
vtkRenderingVolume-8.2.lib
vtkRenderingVolumeOpenGL2-8.2.lib
vtksqlite-8.2.lib
vtksys-8.2.lib
vtktheora-8.2.lib
vtktiff-8.2.lib
vtkverdict-8.2.lib
vtkViewsContext2D-8.2.lib
vtkViewsCore-8.2.lib
vtkViewsInfovis-8.2.lib
vtkzlib-8.2.lib

8.Set the following compilation option to avoid unnecessary compilation errors (same as `demo_online`, set the `_CRT_SECURE_NO_WARNINGS` option):
BOOST_USE_WINDOWS_H
NOMINMAX
_CRT_SECURE_NO_DEPRECATE

##  7.3.4 Compilation and Execution

Compile the `vanjee_driver_viewer` project, set it as the startup item, and run it.