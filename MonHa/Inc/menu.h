#ifndef MENU_H_
#define MENU_H_

typedef struct Menu Pozycja_Menu;

struct Menu{
	// wzorowane na liscie dwustronnej
	const char* Nazwa_Pozycji;
	Pozycja_Menu* Nastepna;
	Pozycja_Menu* Poprzednia;
	Pozycja_Menu* Rodzic;
	Pozycja_Menu* Dziecko;
	// wskaznik na funkcje danej Pozycji
	void (*Funkcja_Pozycji)(void);
};

Pozycja_Menu Start;
Pozycja_Menu Tryb1;
Pozycja_Menu Wyswietl_Info_Tryb1;
Pozycja_Menu Tryb2;
Pozycja_Menu Wyswietl_Info_Tryb2;

void Menu_Odswiez(void);
void Menu_Nast(void);
void Menu_Poprz(void);
void Menu_Funkcja(void);
void Menu_Dziecko(void);
void Menu_Rodzic(void);

#endif /* MENU_H_ */
