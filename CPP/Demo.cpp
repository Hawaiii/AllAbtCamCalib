
#include "AppleTree.h"


using namespace cv;
void parse_argv(int argc, char** argv) {
  if(argc > 2) {
    printf("Incorrect usage: Demo image_dir\n");
    exit(-1);
  }
}


int main(int argc, char** argv) {

  // parse_argv(argc, argv);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // N_shoots(N_shoots = 30),
  // N_poses(N_poses = 2),
  // Lookup_table_dir(a = ""),
  // Image_list_dir(b = ""),
  // output_dir(c = ""),
  // image_prefix(d = ""),
  // pose_prefix(e = "")
  opt option(29,2,"../../calib/h600.csv",
  "../data/",
  "./",
  "image",
  "pose",
  ".png"
);
  std::cout<<"starts from here...\n";
  AppleJuice* juice = new AppleJuice();
  try{
  juice->ReadImageLists(option);
} catch (std::exception &e){
  std::cout << e.what() << std::endl;
}
  juice->ReadLookup_table("");

   return 0;
  }
