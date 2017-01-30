/*
 * writing_to_string.cpp
 *
 *  Created on: 24/4/2015
 *      Author: angeld
 */




#include <io_data_file/Jzon.h>
#include <iostream>
#include <string>
using namespace std;
int main()
{
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

        Jzon::Writer writer(root, Jzon::StandardFormat);
        writer.Write();
        std::string result = writer.GetResult();
        std::cout<<result;
        return 0;
}
