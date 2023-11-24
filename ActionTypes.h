#ifndef __KEY_ACTION_TYPES_H__
#define __KEY_ACTION_TYPES_H__

#include <vector>

class ToggleAction {
public:

    ToggleAction(char key, const char* toggleTopic);

    char getKey() const { return _key; }
    const String& getToggleTopic() const { return _toggleTopic; }

private:
    char _key;
    String _toggleTopic;
};


class MacroAction {
public:

    struct Command {
        const char* topic;
        const char* message;
    };

    typedef std::vector<Command> Commands;

    MacroAction(char key, const Commands& commands);

    char getKey() const { return _key; }

    const Commands& getCommands() const { return _commands; }

private:
    char _key;
    Commands _commands;
};

#endif
