add_subdirectory(bil_dataset_stats)
add_subdirectory(bil_file_viewer)
add_subdirectory(bil_preview_generator)
add_subdirectory(impulse_detector)
add_subdirectory(lcf_file_merger)
add_subdirectory(lcf_file_viewer)
add_subdirectory(exported_ortho_viewer)
add_subdirectory(patch_extractor)
add_subdirectory(aviris4_file_viewer)
add_subdirectory(aviris4_file_converter)
add_subdirectory(PikaL_boresight_calibration_test)
if(${SteviappSdk_FOUND})
add_subdirectory(MalahydSteviappModule)
add_subdirectory(bil_rectifier)
endif()
