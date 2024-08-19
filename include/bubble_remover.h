#ifndef bubble_remover_h
#define bubble_remover_h

#include "config.h"
#include <Arduino.h>

class BubbleRemover {
public: 
    BubbleRemover();

    bool is_bubble();
    void start(Regime& regime_state);
    void stop(Regime& regime_state);
};

#endif