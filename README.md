This is our own implementation of paper:
G. Mattyus and F. Fraundorfer, “Aerial image sequence geolocalization 
with road traffic as invariant feature,” Image and Vision Computing,
vol. 52, pp. 218–229, 2016.

There are some differences between our implementation and the the original paper:
1) We use a road detector instead of a car tracker to find the road;
2) We use only road intersections to construct the basis in both database construction satge
   and shortlist retrieval stage;
3) We use single thread instead of multiple threads.


------------------------------------------------------
Build
The project depends on OpenCV and the shapelib(used to deal with the road vector map files which are save in .shp format ).
Your should replace the opencv path in the top cmakelist file. After that, follow the standard cmake procedure:
 $ mkdir build
 $ cd build
 $ cmake ../
 $ make ./


------------------------------------------------------
Run
When you finish building the project, the executable file will be generated in the directory 'bin'. You can run the geolocalizetion 
with:
    auto_geolocalize [shp_file] [query_imgs_path] [result_file] [result_img_save_path]
Here,
    shp_file --- The reference road map saved in .shp format;
    query_imgs_path --- The query road map saved in .png format. This can be get with a road detector.
    result_file --- The file to save the localization result. Refer to the auto_geolocalize.cc for the format in details.
    result_img_save_path --- Directory to hold the localization result images where the query road map are projected onto the reference map.
                            This is used to verify the correctness of the geolocalization.
