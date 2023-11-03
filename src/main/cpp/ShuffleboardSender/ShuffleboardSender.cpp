#include "ShuffleboardSender\ShuffleboardSender.h"

ShuffleboardSender::ShuffleboardSender(std::string name, bool enabled):
    name_(name),
    enabled_(enabled)
{
    if(enabled){
        tab_ = &frc::Shuffleboard::GetTab(name_);
    }
}

void ShuffleboardSender::update(bool edit){
    if(enabled_){
        for(ShuffleboardItemInterface* item : items_){
            item->update(edit);
        }
    }
};

void ShuffleboardSender::enable(){
    enabled_ = true;
    for(ShuffleboardItemInterface* item : items_){
        item->enable();
    }
}

void ShuffleboardSender::disable(){
    enabled_ = false;
    for(ShuffleboardItemInterface* item : items_){
        item->disable();
    }
}