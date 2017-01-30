/*
 * writing_json_file.cpp
 *
 *  Created on: 24/4/2015
 *      Author: angeld
 */
#include <io_data_file/Jzon.h>
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char * argv[])
{
	if (argc==2){
		Jzon::Object root;
		root.Add("name", "value");
		root.Add("number", 20);
		root.Add("anothernumber", 15.3);
		root.Add("stuff", true);

		Jzon::Array listOfStuff;
		listOfStuff.Add("json");
		listOfStuff.Add("more stuff");
		listOfStuff.Add(Jzon::null);
		listOfStuff.Add(false);
		root.Add("listOfStuff", listOfStuff);
		string path(argv[1]);
		Jzon::FileWriter::WriteFile(path, root, Jzon::StandardFormat);
	}else{
		cout <<" It must be exist one parameter with the path to store the json file "<<endl;
		return -1;
	}
	return 0;
}



