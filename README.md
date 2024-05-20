## ros2_psd_pcb_reloc

A better writeup is to follow in the coming months


## TODOs

- [ ] Better introduction line shown in the top-left

- [ ] Update README.md file with instructions on downloading and building files [July 2024]

- [ ] On Zenodo only publish LSU_iCORE_MONO dataset with instructions on how to download and setup the EuRoC and TUM FR2 dataset [ June 2024]

- [ ] Delete all package 2 related codes, ros2_tictoc_profiler, line_lbd, fast_mf [05/20/24]

- [ ] Release ros2_psd_pcb_reloc public [July 18th 2024]

- [x] Build and test the packages [05/20/24]

* Make sure to mention that dataset names must be in all caps

* Make sure to mention that the camera intrinsic matrice YAML may be different from ORB SLAM3 V1

* make sure to state only pinhole camera models was tested in this framework

* For making a custom dataset work with the YOLOv5 in py_obj_detector, custom yaml files are needed. State how to add them into the /global_yaml files . This information needs to be placed ion the ```ros2_psd_pcb``` repository
