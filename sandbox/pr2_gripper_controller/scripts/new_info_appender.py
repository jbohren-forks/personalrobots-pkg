text_file_list = open("text_list3.txt","r")
grasp_data_old = open("grasp_data_new.csv","r")
grasp_data_new = open("grasp_data_appended.csv","w")
grasp_data_new.write("object, amount, state, brand, result, velocity, effort, first_contact_distance, fingertip_peak_force_left, fingertip_peak_force_right, fingertip_steady_force_left, fingertip_steady_force_right, steady_state_distance, peak_force_distance,bag_name,dt,source\n")
current_file = text_file_list.readline()
while (current_file != ""):
  current_file = current_file.strip().replace(".txt", "-fast.txt")
  text_file = open(current_file,"r")
  current_line = text_file.readline()
  last_line = current_line
  while (current_line != ""):
    last_line = current_line
    current_line = text_file.readline()
  last_line_arr = last_line.split(',')
  grasp_data_new.write("%s,%s,%s\n" %(grasp_data_old.readline().strip().replace('"','').replace('&',','),last_line_arr[4], current_file))
  print "%s" %current_file
  text_file.close()
  current_file = text_file_list.readline()
grasp_data_new.close()
grasp_data_old.close()