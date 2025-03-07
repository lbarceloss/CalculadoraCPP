#pragma once

class PowerOptions {
public:
    int Auxpart;
    int Mascot;
    int Card;
    int PsAuxpart;
    int PsMascot;
    int PsCard;

    // Construtor
    PowerOptions(int auxpart = 0, int mascot = 0, int card = 0, int psAuxpart = 0, int psMascot = 0, int psCard = 0)
        : Auxpart(auxpart), Mascot(mascot), Card(card), PsAuxpart(psAuxpart), PsMascot(psMascot), PsCard(psCard) {
    }

    // Método Total equivalente ao C#
    int Total(int option) const {
        int pwr = Auxpart + Mascot + Card;
        if (option == 1 || option == 2 || option == 3) {
            pwr += PsAuxpart + PsMascot + PsCard;
        }
        return pwr;
    }
};
