/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GZ_SIM_TUNINGPLUGIN_HH_
#define GZ_SIM_TUNINGPLUGIN_HH_

#include <gz/sim/gui/GuiSystem.hh>

class TuningPlugin : public gz::sim::GuiSystem
{
  Q_OBJECT

  Q_PROPERTY(double number MEMBER number NOTIFY TuningGainsChanged)

  public: TuningPlugin();
  public: ~TuningPlugin() override;

  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  public: void Update(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  signals: void TuningGainsChanged();


  private: double number{1.0};

  };

#endif
