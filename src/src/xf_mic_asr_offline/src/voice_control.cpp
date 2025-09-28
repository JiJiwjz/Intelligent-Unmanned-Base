/*******************************************************
 This contents of this file may be used by anyone
 for any reason without any conditions and may be
 used as a starting point for your own applications
 which use HIDAPI.
********************************************************/
#include <user_interface.h>
#include <string>
#include <locale>
#include <codecvt>
#include <ctime>
#include <joint.h>
#include <record.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <xf_mic_asr_offline/BuzzerState.h>
#include <xf_mic_asr_offline/GetOfflineResult.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <sys/stat.h>

ros::Publisher voice_words_pub;
ros::Publisher buzzer_pub;
int offline_recognise_switch = 0; //离线识别默认开关(default switch for offline recognition)

using namespace std;

extern UserData asr_data;
extern int whether_finised ;
extern char *whole_result;
int set_led_id ;

extern int init_rec;
extern int init_success;
extern int write_first_data;

unsigned char* record_data;

const char str_ = '|';
const char str_none = ' ';
bool shouldExit = false; //yct add: 用于终止语音程序

std::wstring s2ws(const std::string &str)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.from_bytes(str);
}

std::string ws2s(const std::wstring &wstr)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.to_bytes(wstr);
}

/*用于送入音频进行识别 (line)*/
int business_data(unsigned char* record)
{
    record_data = record;
    if (!init_success && init_rec)
    {
        int len = 3*PCM_MSG_LEN;
        char *pcm_buffer=(char *)malloc(len);
        if (NULL == pcm_buffer)
            {
                printf(">>>>>buffer is null\n");
                exit (1);
            }
        memcpy(pcm_buffer, record_data, len);

        if (write_first_data++ == 0)
        {
    #if whether_print_log
            printf("***************write the first voice**********\n");
    #endif
            demo_xf_mic(pcm_buffer, len, 1);
        }

        else
        {
    #if whether_print_log
            printf("***************write the middle voice**********\n");
    #endif
            demo_xf_mic(pcm_buffer, len, 2);
        }
        if (whether_finised)
        {
            record_finish = 1;
            whether_finised = 0;
        }
    }   
}


/*用于显示离线命令词识别结果(used to display the offline recognition result of the voice command)*/
Effective_Result show_result(char *string) //
{
	Effective_Result current;
	if (strlen(string) > 250)
	{
		char asr_result[32];	//识别到的关键字的结果(the result of the recognized keyword)
		char asr_confidence[3]; //识别到的关键字的置信度(the confidence of the recognized keyword)
		char *p1 = strstr(string, "<focus>");
		char *p2 = strstr(string, "</focus>");
		int n1 = p1 - string + 1;
		int n2 = p2 - string + 1;

		char *p3 = strstr(string, "<confidence>");
		char *p4 = strstr(string, "</confidence>");
		int n3 = p3 - string + 1;
		int n4 = p4 - string + 1;
		for (int i = 0; i < 32; i++)
		{
			asr_result[i] = '\0';
		}

		strncpy(asr_confidence, string + n3 + strlen("<confidence>") - 1, n4 - n3 - strlen("<confidence>"));
		asr_confidence[n4 - n3 - strlen("<confidence>")] = '\0';
		int confidence_int = 0;
		confidence_int = atoi(asr_confidence);
		if (confidence_int >= confidence)
		{
			strncpy(asr_result, string + n1 + strlen("<focus>") - 1, n2 - n1 - strlen("<focus>"));
			asr_result[n2 - n1 - strlen("<focus>")] = '\0'; //加上字符串结束符。(add string terminator)
		}
		else
		{
			strncpy(asr_result, "", 0);
		}

		current.effective_confidence = confidence_int;
        std::replace(std::begin(asr_result), std::end(asr_result), str_, str_none);
        strcpy(current.effective_word, asr_result);
		return current;
	}
	else
	{
		current.effective_confidence = 0;
		strcpy(current.effective_word, " ");
		return current;
	}
}

//语音识别请你闭嘴
bool ShutUpCallBack(){
	shouldExit = true;
	printf(">>>>>语音节点关闭\n");
	ros::shutdown();
	return true;
}

/*yct add: 避免无用功，这里假装识别一次，实际上说啥都是一样的 */
/*yct add: 如果你要用正经的语音识别，就把"get_offline_result"的回调函数替换为"Get_Offline_Recognise_Result" */
bool Fake_Recognise_Result(xf_mic_asr_offline::GetOfflineResult::Request &req,
								  xf_mic_asr_offline::GetOfflineResult::Response &res)
{
	char *denoise_sound_path = join(source_path, DENOISE_SOUND_PATH);
	printf(">>>>>开始一次语音识别！(start first voice recognition!)\n");
	xf_mic_asr_offline::BuzzerState buzzer;
	buzzer.freq = 3000;
	buzzer.on_time = 0.05;
	buzzer.off_time = 0.01;
	buzzer.repeat = 1;
	buzzer_pub.publish(buzzer);
	sleep(3.0);
	whole_result = "<?xml version='1.0' encoding='utf-8' standalone='yes' ?><nlp>  <version>1.1</version>  <rawtext>开始执行任务</rawtext>  <confidence>15</confidence>  <engine>local</engine>  <result>    <focus>开始执行任务</focus>    <confidence>14</confidence>    <object>      <开始执行任务 id=\"65535\">开始执行任务</开始执行任务>    </object>  </result></nlp>";
	Effective_Result effective_ans = show_result(whole_result);
	if (effective_ans.effective_confidence >= confidence) //如果大于置信度阈值则进行显示或者其他控制操作(if it is greater than the confidence threshold, display or other control operation will be performed)
	{
		printf(">>>>>是否识别成功(whether the recognition succeeds): [ %s ]\n", "是");
		printf(">>>>>关键字的置信度(keywors confidence): [ %d ]\n", effective_ans.effective_confidence);
		printf(">>>>>关键字识别结果(keyword recognition result): [ %s ]\n", effective_ans.effective_word);
		/*发布结果(publish the result)*/
		//control_jetbot(effective_ans.effective_word);
		res.result = "ok";
		res.fail_reason = "";
		std::wstring wtxt = s2ws(effective_ans.effective_word);
		std::string txt_uft8 = ws2s(wtxt);
		res.text = txt_uft8;
		
		std_msgs::String msg;
		msg.data = effective_ans.effective_word;
		voice_words_pub.publish(msg);
	}
	whole_result = "";
	/*[1-3]语音识别结束]([1-3] voice recognition end)*/
	delete_asr_engine();
	write_first_data = 0;
	sleep(1.0);
	printf(">>>>>完成一次识别(finish voice recognition)\n");
	printf(" \n");
	return true;

}


/*获取离线命令词识别结果(acquire the offline recognition result of the voice command)*/
bool Get_Offline_Recognise_Result(xf_mic_asr_offline::GetOfflineResult::Request &req,
								  xf_mic_asr_offline::GetOfflineResult::Response &res)
{
	char *denoise_sound_path = join(source_path, DENOISE_SOUND_PATH);
	offline_recognise_switch = req.offline_recognise_start;
	if (offline_recognise_switch == 1) //如果是离线识别模式(if it is the offline recognition mode)
	{
		whether_finised = 0;
		record_finish = 0;
		int ret = 0;
		ret = create_asr_engine(&asr_data);
		if (MSP_SUCCESS != ret)
		{
#if whether_print_log
			printf("[01]创建语音识别引擎失败！(fail to create voice recognition engine)\n");
#endif
		}

		printf(">>>>>开始一次语音识别！(start first voice recognition!)\n");
        xf_mic_asr_offline::BuzzerState buzzer;
        buzzer.freq = 3000;
        buzzer.on_time = 0.05;
        buzzer.off_time = 0.01;
        buzzer.repeat = 1;
        buzzer_pub.publish(buzzer);
        get_the_record_sound(denoise_sound_path);

		if (whole_result!="")
		{
			printf(">>>>>全部返回结果:　[ %s ]\n", whole_result);
			Effective_Result effective_ans = show_result(whole_result);
			if (effective_ans.effective_confidence >= confidence) //如果大于置信度阈值则进行显示或者其他控制操作(if it is greater than the confidence threshold, display or other control operation will be performed)
			{
				printf(">>>>>是否识别成功(whether the recognition succeeds): [ %s ]\n", "是");
				printf(">>>>>关键字的置信度(keywors confidence): [ %d ]\n", effective_ans.effective_confidence);
				printf(">>>>>关键字识别结果(keyword recognition result): [ %s ]\n", effective_ans.effective_word);
				/*发布结果(publish the result)*/
				//control_jetbot(effective_ans.effective_word);
				res.result = "ok";
				res.fail_reason = "";
				std::wstring wtxt = s2ws(effective_ans.effective_word);
				std::string txt_uft8 = ws2s(wtxt);
				res.text = txt_uft8;
				
				std_msgs::String msg;
				msg.data = effective_ans.effective_word;
				voice_words_pub.publish(msg);
			}
			else
			{
				printf(">>>>>是否识别成功(whether the recognition succeeds): [ %s ]\n", "否");
				printf(">>>>>关键字的置信度(keywords confidence): [ %d ]\n", effective_ans.effective_confidence);
				printf(">>>>>关键字置信度较低，文本不予显示(keyword confidence is too low. The text will not be displayed)\n");
				res.result = "fail";
				res.fail_reason = "low_confidence error or 11212_license_expired_error";
				res.text = " ";
			}
		}
		else
		{
			res.result = "fail";
			res.fail_reason = "no_valid_sound error";
			res.text = " ";
			printf(">>>>>未能检测到有效声音,请重试(no sound is detected, and please try again)\n");
		}
		whole_result = "";
		/*[1-3]语音识别结束]([1-3] voice recognition end)*/
		delete_asr_engine();
		write_first_data = 0;
		sleep(1.0);
	}
	printf(">>>>>完成一次识别(finish voice recognition)\n");
	printf(" \n");
	//ROS_INFO("close the offline recognise mode ...\n");
	return true;
}

/*程序入口(entry to the program)*/
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "voice_control");
	ros::NodeHandle ndHandle("~");
	ndHandle.param("confidence", confidence, 0);//离线命令词识别置信度阈值(offline recognition confidence threshold of the voice command)
	ndHandle.param("seconds_per_order", time_per_order, 5); //单次录制音频的时长(time taken to record single audio)
	ndHandle.param("source_path", source_path, std::string(""));
	ndHandle.param("appid", appid, std::string(""));//appid，需要更换为自己的(appid. It should be changed to your own)

	printf(">>>>>confidence = %d\n",confidence);
	printf(">>>>>time_per_order = %d\n",time_per_order);

	cout<<">>>>>source_path = "<<source_path<<endl;
	cout<<">>>>>appid = "<<appid<<endl;

	APPID = &appid[0];

	voice_words_pub = ndHandle.advertise<std_msgs::String>("voice_words", 1);
    buzzer_pub = ndHandle.advertise<xf_mic_asr_offline::BuzzerState>("/ros_robot_controller/set_buzzer", 1);

	/*srv　接收请求，返回离线命令词识别结果(srv　receive the request. Return the offline recognition result of the voice command)*/
	ros::ServiceServer service_get_wav_list = ndHandle.advertiseService("get_offline_result", Fake_Recognise_Result);
	//ros::ServiceServer get_shut_up = ndHandle.advertiseService("shut_up_mother_fucker", ShutUpCallBack); //yct add: yct定义的终止语音识别程序
	//yct add: 最后还是选择了直接在python中kill这个节点（太不优雅了）

	std::string begin = "fo|";
	//std::string quit_begin = source_path;
	char *jet_path = join((begin + source_path), ASR_RES_PATH);
	char *grammer_path = join(source_path, GRM_BUILD_PATH);
	char *bnf_path = join(source_path, GRM_FILE);

	Recognise_Result inital = initial_asr_paramers(jet_path, grammer_path, bnf_path, LEX_NAME);

	init_rec = 0;
	init_success = 0;
	write_first_data = 0;
	
	ros::AsyncSpinner spinner(3);
	spinner.start();
        
	ndHandle.setParam("init_finish", true);
	while(ros::ok())
	{
	    if (!init_success && init_rec)
	    {
	        //获取当前时间
	        clock_t start, finish;
	        double total_time;
	        start = clock();
	        while (!init_success && whether_finised != 1)
	        {   
	            finish = clock();
	            total_time = (double)(finish - start) / CLOCKS_PER_SEC/2;
	            //printf(">>>>>total_time:　[ %f ]\n", total_time);
	            //printf(">>>>>whether_finised:　[ %d ]\n", whether_finised);
	            if (total_time > time_per_order)
	            {
	                printf(">>>>>超出离线命令词最长识别时间(exceeds the maximum recognition time of offline voice command)\n");
	                record_finish = 1; 
	                break;
	            }
	        } 
	    }		
		
	} 
	ros::spinOnce(); 
	return 0;
}
