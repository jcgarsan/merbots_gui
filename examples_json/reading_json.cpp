/*
 * reading_json.cpp
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
		string path(argv[0]);
		Jzon::Object rootNode;
		Jzon::FileReader::ReadFile("file.json", rootNode);

		for (Jzon::Object::iterator it = rootNode.begin(); it != rootNode.end(); ++it)
		{
			std::string name = (*it).first;
			Jzon::Node &node = (*it).second;

			std::cout << name << " = ";
			if (node.IsValue())
			{
				switch (node.AsValue().GetValueType())
				{
				case Jzon::Value::VT_NULL   : std::cout << "null"; break;
				case Jzon::Value::VT_STRING : std::cout << node.ToString(); break;
				case Jzon::Value::VT_NUMBER : std::cout << node.ToFloat(); break;
				case Jzon::Value::VT_BOOL   : std::cout << (node.ToBool()?"true":"false"); break;
				}
			}
			else if (node.IsArray())
			{
				std::cout << "*Array*";
			}
			else if (node.IsObject())
			{
				std::cout << "*Object*";
			}
			std::cout << std::endl;
		}

		const Jzon::Array &stuff = rootNode.Get("listOfStuff").AsArray();
		for (Jzon::Array::const_iterator it = stuff.begin(); it != stuff.end(); ++it)
		{
			std::cout << (*it).ToString() << std::endl;
		}
	}else
	{
		cout <<" It must be exist one parameter with the path to the json file.json for example"<<endl;
		return -1;
	}
	return 0;
}


