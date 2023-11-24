#include <Arduino.h>
#include "ActionTypes.h"

ToggleAction::ToggleAction(char key, const char* toggleTopic) :
    _key(key),
    _toggleTopic(toggleTopic) 
{

}

MacroAction::MacroAction(char key, const Commands& commands) :
    _key(key),
    _commands(commands)
{
}