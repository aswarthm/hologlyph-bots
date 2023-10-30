rm hb_task_2a_1036.db3
rm hb_task_2a_1036.yaml
rm hb_task_2a_1036.zip
rm task2a_submission.zip

mv rosbag2*/* ./

mv *.db3 ./hb_task_2a_1036.db3
mv *.yaml ./hb_task_2a_1036.yaml

rm rosbag2* -r
