#include "utils.hpp"

using namespace std;

list<ConfigLine> GetConfigLines()
{
    size_t found;
    list<ConfigLine> ConfigLines;
    ConfigLine current;
    ifstream cFile(CONFIG_FILE_LOCATION);
    if (cFile.is_open())
    {
        string line;
        while (getline(cFile, line))
        {
            found = line.find(" ");
            if (found != string::npos)
                line.erase(found, 1);
            if (line[0] == '#' || line.empty())
                continue;
            found = line.find(" ");
            while (found != string::npos)
            {
                line.erase(found, 1);
                found = line.find(" ");
            }
            auto delimiterPos = line.find("=");
            auto name = line.substr(0, delimiterPos);
            auto value = line.substr(delimiterPos + 1);
            found = value.find("\r");
            if (found != string::npos)
                value.erase(found);
            current.name = name;
            current.value = value;
            ConfigLines.push_back(current);
        }
    }
    else
    {
        string msg = "Could not open file, exiting now...";
        cout << "[ERROR] " + msg << endl;
        Log(msg, ERROR);
        exit(1);
    }
    cFile.close();
    return ConfigLines;
}

string GetParam(string identifier)
{
    for (ConfigLine &line : GetConfigLines())
    {
        if (line.name.compare(identifier) == 0)
            return line.value;
    }
    return string("");
}

void WriteConfigLines(list<ConfigLine> lines)
{
    ofstream cFile(CONFIG_FILE_LOCATION);
    string line;
    if (cFile.is_open())
    {
        Log("Writing new configuration...", INFO);
        for (ConfigLine &line : lines)
        {
            cFile << line.name + "=" + line.value << endl;
        }
    }
    else
    {
        string msg = "Could not open file, exiting now...";
        cout << "[ERROR] " + msg << endl;
        Log(msg, ERROR);
        exit(1);
    }
    cFile.close();
}

bool ci_find_substr(std::string str1, std::string str2)
{
    std::transform(str1.begin(), str1.end(), str1.begin(), ::tolower);
    std::transform(str2.begin(), str2.end(), str2.begin(), ::tolower);
    if (str1.find(str2) != string::npos)
        return true;
    return false;
}
