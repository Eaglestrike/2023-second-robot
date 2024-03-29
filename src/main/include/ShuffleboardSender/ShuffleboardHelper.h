#pragma once

#include <frc/shuffleboard/Shuffleboard.h>

#include "ShuffleboardItemInterface.h"

namespace ShuffleboardHelper{
    template <typename T> nt::GenericEntry* createItem(ShuffleboardItemInterface::ItemData data, T value, frc::BuiltInWidgets type = frc::BuiltInWidgets::kTextView){
        // for(auto component : data.tab->GetComponents()){
        //     if(component.get()->GetTitle() == data.name){
        //         nt::NetworkTableInstance::GetDefault().GetEntry(data.name).Delete();
        //     }
        // }
        if((data.pose.positionX >= 0) && (data.pose.positionY >= 0)){
            return data.tab->Add(data.name, value)
                                .WithWidget(type)
                                .WithSize(data.pose.width, data.pose.height)
                                .WithPosition(data.pose.positionX, data.pose.positionY)
                                .GetEntry();
        } 
        else{
            return data.tab->Add(data.name, value)
                                .WithWidget(type)
                                .WithSize(data.pose.width, data.pose.height)
                                .GetEntry();
        }
    }

    inline frc::ShuffleboardLayout* createList(ShuffleboardItemInterface::ItemData data){
        if((data.pose.positionX >= 0) && (data.pose.positionY >= 0)){
            return &data.tab->GetLayout(data.name, frc::BuiltInLayouts::kList)
                                .WithSize(data.pose.width, data.pose.height)
                                .WithPosition(data.pose.positionX, data.pose.positionY)
                                .WithProperties({std::make_pair("Label Position", nt::Value::MakeString("Right"))});
        }
        else{
            return &data.tab->GetLayout(data.name, frc::BuiltInLayouts::kList)
                                .WithSize(data.pose.width, data.pose.height)
                                .WithProperties({std::make_pair("Label Position", nt::Value::MakeString("Right"))});
        }
    }
}