#include <string.h>
#include <stdio.h>
#include "menu.h"
#include "main.h"
#include "stm32l476g_discovery_glass_lcd.h"

// 						Sciaga z elementow struktury
//struct Menu{
//	// wzorowane na liscie dwustronnej
//	const uint8_t* Nazwa_Pozycji;
//	const Pozycja_Menu* Nastepna;
//	const Pozycja_Menu* Poprzednia;
//	const Pozycja_Menu* Rodzic;
//	const Pozycja_Menu* Dziecko;
//	// wskaznik na funkcje danej Pozycji
//	void (*Funkcja_Pozycji)(void);
//};


Pozycja_Menu Start = {"LOGGER", &Tryb1, &Tryb2, NULL, NULL, NULL};
Pozycja_Menu Tryb1 = {"PROG", &Tryb2, &Start, NULL, &Wyswietl_Info_Tryb1, Prog_callback};
Pozycja_Menu Wyswietl_Info_Tryb1 = {"Wyswietl Prog ", NULL, NULL, &Tryb1, NULL, Wyswietl_Prog_callback};
//	Menu Wyswietl_Info_T1 = {"Wyswietl Prog"};
Pozycja_Menu Tryb2 = {"PROBKI", &Start, &Tryb1, NULL, NULL, Probki_callback};
//	Menu Wyswietl_Info_T1;

Pozycja_Menu *AktualnaPozycja = &Start;

void Menu_Odswiez(void){
	int ileZnakow = strlen(AktualnaPozycja->Nazwa_Pozycji);
	if(ileZnakow < 7){  // LCD ma 6 znakow
		BSP_LCD_GLASS_DisplayString(AktualnaPozycja->Nazwa_Pozycji);
	} else {
		BSP_LCD_GLASS_ScrollSentence(AktualnaPozycja->Nazwa_Pozycji, 1, 200);
	}
}

void Menu_Nast(void){
	if(AktualnaPozycja->Nastepna){
		AktualnaPozycja = AktualnaPozycja->Nastepna;
	}
	Menu_Odswiez();
}

void Menu_Poprz(void){
	if(AktualnaPozycja->Poprzednia){
		AktualnaPozycja = AktualnaPozycja->Poprzednia;
	}
	Menu_Odswiez();
}

void Menu_Dziecko(void){
	if(AktualnaPozycja->Dziecko){
		AktualnaPozycja = AktualnaPozycja->Dziecko;
	}
	Menu_Odswiez();
}

void Menu_Rodzic(void){
	if(AktualnaPozycja->Rodzic){
		AktualnaPozycja = AktualnaPozycja->Rodzic;
	}
	Menu_Odswiez();
}

void Menu_Funkcja(void){
	if (AktualnaPozycja->Funkcja_Pozycji){
		AktualnaPozycja->Funkcja_Pozycji();
	}
	Menu_Odswiez();
}
