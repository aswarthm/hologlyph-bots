rm hb_task_2b_1036.db3
rm hb_task_2b_1036.yaml
rm hb_task_2b_1036.zip
rm task2b_submission.zip

mv rosbag2*/* ./

mv *.db3 ./hb_task_2b_1036.db3
mv *.yaml ./hb_task_2b_1036.yaml

rm rosbag2* -r
