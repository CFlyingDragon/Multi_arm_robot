#include <kdl_parser/kdl_parser.hpp>
KDL::Tree my_tree;
   urdf::Model my_model;
   if (!my_model.initXml(....)){
      ROS_ERROR("Failed to parse urdf robot model");
      return false;
   }
   if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }