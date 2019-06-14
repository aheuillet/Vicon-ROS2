#include "utils.hpp"

using namespace std;


string GetParam(string config_file, string identifier){
    size_t found;
    ifstream cFile (config_file);
    if (cFile.is_open())
    {
        string line;
        while(getline(cFile, line)){
            found = line.find(" ");
            if (found != string::npos)
                line.erase(found, 1);
            if(line[0] == '#' || line.empty())
                continue;
            found = line.find(" ");
            while(found != string::npos){
                line.erase(found, 1);
                found = line.find(" ");
            }
            auto delimiterPos = line.find("=");
            auto name = line.substr(0, delimiterPos);
            auto value = line.substr(delimiterPos + 1);
            found = value.find("\r");
            if (found != string::npos)
                value.erase(found);
            if (name.compare(identifier)==0)
                return value;
        }
    }
    return string("");
}
