// This file contains old preliminary tests of the correlation matrices/DNFs

  void test_dnf_setup(){
    // Test: Setup the DNFs with predefined values
    
    // - - - - - - - - - - - - - - - - - - - Keyword x Action test- - - - - - - - - - - - - - - - - - - 
    // Learning / Memorization phase - - - - - - - - 
    // Learning "Move left"
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(0, 6.9f); // The keyword "Move"
    keywords_dnf.set_input_element(5, 6.9f); // The keyword "left"
    action_dnf.set_input_element(MOVE_LEFT, 6.9f); // The action "Move left"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input(), 0.5f); //Get it into memory
    RCLCPP_INFO(this->get_logger(), "Memorized Move left");
    // Learning "Move right"
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(0, 6.9f); // The keyword "Move"
    keywords_dnf.set_input_element(6, 6.9f); // The keyword "right"
    action_dnf.set_input_element(MOVE_RIGHT, 6.9f); // The action "Move right"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input(), 0.5f); //Get it into memory
    RCLCPP_INFO(this->get_logger(), "Memorized Move right");
    // Learning "Move back"
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(0, 6.9f); // The keyword "Move"
    keywords_dnf.set_input_element(10, 6.9f); // The keyword "away"
    action_dnf.set_input_element(MOVE_BACK, 6.9f); // The action "Move forward"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input(), 0.5f); //Get it into memory
    RCLCPP_INFO(this->get_logger(), "Memorized Move forward");
    // Learning "Move forward"
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(0, 6.9f); // The keyword "Move"
    keywords_dnf.set_input_element(9, 6.9f); // The keyword "closer"
    action_dnf.set_input_element(MOVE_FORWARD, 6.9f); // The action "Move forward"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input(), 0.5f); //Get it into memory
    RCLCPP_INFO(this->get_logger(), "Memorized Move forward");

    // Learning "Release"
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(17, 6.9f); // The keyword "Release"
    action_dnf.set_input_element(RELEASE, 6.9f); // The action "Release"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input(), 0.5f); //Get it into memory
    RCLCPP_INFO(this->get_logger(), "Memorized Release");
    // Learning "Grasp"
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(18, 6.9f); // The keyword "Grasp"
    action_dnf.set_input_element(GRASP, 6.9f); // The action "Grasp"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input(), 0.5f); //Get it into memory
    RCLCPP_INFO(this->get_logger(), "Memorized Grasp");
    // Learning "Pick" (Up)
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(19, 6.9f); // The keyword "Pick"
    action_dnf.set_input_element(PICK_UP, 6.9f); // The action "Pick Up"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input(), 0.5f); //Get it into memory
    RCLCPP_INFO(this->get_logger(), "Memorized Pick Up");
    // Learning "Place" (Down)
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(20, 6.9f); // The keyword "Place"
    action_dnf.set_input_element(PLACE_DOWN, 6.9f); // The action "Place Down"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    RCLCPP_INFO(this->get_logger(), "Memorized Place Down");
    // Save KWxA DNF after learning
    write2DTensorCSV(keywords_action_dnf.get_activation(), log_path, "keywords_action_dnf_activation_after_learning.csv");
    RCLCPP_INFO(this->get_logger(), "Saved KWxA DNF after learning");
    // // Memory extraction / Remembering phase - - - - - - - -
    // // Remembering move
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(0, 6.9f); //"Move"
    // torch::Tensor move_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(move_response, log_path, "move_response.csv");

    // // Remembering release
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(21, 6.9f); //"Release"
    // torch::Tensor release_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(release_response, log_path, "release_response.csv");

    // // Remembering GRASP
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(22, 6.9f); //"grasp"
    // torch::Tensor grasp_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(grasp_response, log_path, "grasp_response.csv");

    // // Remembering "Pick"
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(23, 6.9f); //"Pick"
    // torch::Tensor pick_reponse = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(pick_reponse, log_path, "pick_response.csv");

    // // Remembering "Place"
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(24, 6.9f); //"Place"
    // torch::Tensor place_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(place_response, log_path, "place_response.csv");

    // // Remember / Extract for a word that has not been learned
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(17, 6.9f); //"Away"
    // torch::Tensor dummy_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(dummy_response, log_path, "dummy_response.csv");

    // // Remember / Extract for several words that have been learned
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(0, 6.9f); //"Move"
    // keywords_dnf.set_input_element(21, 6.9f); //"Release"
    // torch::Tensor red_green_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(red_green_response, log_path, "move_release_response.csv");

    // // Remember / Extract for a phrase containing one learned word
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(0, 6.9f); //"Move"
    // keywords_dnf.set_input_element(1, 6.9f); //"The"
    // keywords_dnf.set_input_element(4, 6.9f); //"Red"
    // keywords_dnf.set_input_element(2, 6.9f); //"Ball"
    // keywords_dnf.set_input_element(8, 6.9f); //"Right"
    // torch::Tensor red_phrase_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(red_phrase_response, log_path, "move_phrase_response.csv");

    // - - - - - - - - - - - - - - - - - - - Keyword x Color test 2 - - - - - - - - - - - - - - - - - - - 
    // // Learning / Memorization phase - - - - - - - - 
    // // Learning red
    keywords_dnf.reset_input();
    color_circles_dnf.reset_input();
    keywords_dnf.set_input_element(4, 6.9f); //"Red"
    color_circles_dnf.set_input_element(0, 6.9f); // The red color
    keywords_color_dnf.set_input(keywords_dnf.get_input(), color_circles_dnf.get_input()); //Set input for the 2D DNF
    keywords_color_dnf.step(keywords_dnf.get_input(), color_circles_dnf.get_input(), 0.5f); //Get it into memory
    RCLCPP_INFO(this->get_logger(), "Learned red");
    // Learning green
    keywords_dnf.reset_input();
    color_circles_dnf.reset_input();
    keywords_dnf.set_input_element(5, 6.9f);
    color_circles_dnf.set_input_element(1, 6.9f); // The green color
    keywords_color_dnf.set_input(keywords_dnf.get_input(), color_circles_dnf.get_input());
    keywords_color_dnf.step(keywords_dnf.get_input(), color_circles_dnf.get_input(), 0.5f); //Get it into memory
    RCLCPP_INFO(this->get_logger(), "Learned green");
    // Learning blue
    keywords_dnf.reset_input();
    color_circles_dnf.reset_input();
    keywords_dnf.set_input_element(6, 6.9f);
    color_circles_dnf.set_input_element(2, 6.9f); // The blue color
    keywords_color_dnf.set_input(keywords_dnf.get_input(), color_circles_dnf.get_input());
    keywords_color_dnf.step(keywords_dnf.get_input(), color_circles_dnf.get_input(), 0.5f); //Get it into memory
    RCLCPP_INFO(this->get_logger(), "Learned blue");
    // Save KWxC DNF after learning
    write2DTensorCSV(keywords_color_dnf.get_activation(), log_path, "keywords_color_dnf_activation_after_learning.csv");

    // // Memory extraction / Remembering phase - - - - - - - -
    // // Remembering red
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(4, 6.9f); //"Red"
    // torch::Tensor red_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(red_response, log_path, "red_response.csv");

    // // Remembering green
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(5, 6.9f); //"Green"
    // torch::Tensor green_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(green_response, log_path, "green_response.csv");

    // // Remembering blue
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(6, 6.9f); //"Blue"
    // torch::Tensor blue_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(blue_response, log_path, "blue_response.csv");

    // // Remember / Extract for a word that has not been learned
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(17, 6.9f); //"Away"
    // torch::Tensor dummy_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(dummy_response, log_path, "dummy_response.csv");

    // // Remember / Extract for several words that have been learned
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(4, 6.9f); //"Red"
    // keywords_dnf.set_input_element(5, 6.9f); //"Green"
    // torch::Tensor red_green_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(red_green_response, log_path, "red_green_response.csv");

    // // Remember / Extract for a phrase containing one learned word
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(0, 6.9f); //"Move"
    // keywords_dnf.set_input_element(1, 6.9f); //"The"
    // keywords_dnf.set_input_element(4, 6.9f); //"Red"
    // keywords_dnf.set_input_element(2, 6.9f); //"Ball"
    // keywords_dnf.set_input_element(8, 6.9f); //"Right"
    // torch::Tensor red_phrase_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(red_phrase_response, log_path, "red_phrase_response.csv");
  }