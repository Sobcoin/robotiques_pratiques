#ifndef LABO3_H
#define LABO3_H

// Séquence interne du labo 3
enum stateMachine { START,
                    SEGMENTONE,
                    PIVOTONE,
                    SEGMENTWO,
                    PIVOTWO,
                    SEGMENTHREE,
                    SEMIPIVOT,
                    END
};
extern stateMachine stateRobot;

// Gestionnaire de la séquence du labo 3 (appelé dans loop)
void stateManager(unsigned long ct);

#endif