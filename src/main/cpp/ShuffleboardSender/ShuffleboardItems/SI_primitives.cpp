#include "ShuffleboardSender/ShuffleboardItems/SI_primitives.h"
#include "ShuffleboardSender/ShuffleboardHelper.h"

//Constructor
ShuffleboardItem<double>::ShuffleboardItem(ItemData data, double* value):
    BaseShuffleboardItem(data)
{
    value_ = value;
    entry_ = ShuffleboardHelper::createItem(data, *value);
}

ShuffleboardItem<bool>::ShuffleboardItem(ItemData data, bool* value):
    BaseShuffleboardItem(data)
{
    value_ = value;
    if(data.edit){
        entry_ = ShuffleboardHelper::createItem(data, *value, frc::BuiltInWidgets::kToggleButton);
    }
    else{
        entry_ = ShuffleboardHelper::createItem(data, *value);
    }
}

ShuffleboardItem<int>::ShuffleboardItem(ItemData data, int* value):
    BaseShuffleboardItem(data)
{
    value_ = value;
    entry_ = ShuffleboardHelper::createItem(data, *value);
}

//Send
void ShuffleboardItem<double>::send(){
    entry_->SetDouble(*value_);
}

void ShuffleboardItem<bool>::send(){
    entry_->SetBoolean(*value_);
}

void ShuffleboardItem<int>::send(){
    entry_->SetInteger(*value_);
}

//Edit
void ShuffleboardItem<double>::edit(){
    *value_ = entry_->GetDouble(*value_);
}
void ShuffleboardItem<bool>::edit(){
    *value_ = entry_->GetBoolean(*value_);
}
void ShuffleboardItem<int>::edit(){
    *value_ = entry_->GetInteger(*value_);
}

//Is value updated
bool ShuffleboardItem<double>::itemHasChanged(){
    double newVal = *value_;
    bool hasChanged = (prevVal_ != newVal);
    prevVal_ = newVal;
    return hasChanged;
} 

bool ShuffleboardItem<bool>::itemHasChanged(){
    bool newVal = *value_;
    bool hasChanged = (prevVal_ != newVal);
    prevVal_ = newVal;
    return hasChanged;
} 

bool ShuffleboardItem<int>::itemHasChanged(){
    int newVal = *value_;
    bool hasChanged = (prevVal_ != newVal);
    prevVal_ = newVal;
    return hasChanged;
} 

//Enable
void ShuffleboardItem<double>::enable(){
    if(!entry_->Exists()){
        entry_ = ShuffleboardHelper::createItem(data_, *value_);
    }
};
void ShuffleboardItem<bool>::enable(){
    if(!entry_->Exists()){
        entry_ = ShuffleboardHelper::createItem(data_, *value_);
    }
};
void ShuffleboardItem<int>::enable(){
    if(!entry_->Exists()){
        entry_ = ShuffleboardHelper::createItem(data_, *value_);
    }
};

//Disable
void ShuffleboardItem<double>::disable(){
    entry_->Unpublish();
};
void ShuffleboardItem<bool>::disable(){
    entry_->Unpublish();
};
void ShuffleboardItem<int>::disable(){
    entry_->Unpublish();
};