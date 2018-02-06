//	Библиотека для работы с ультразвуковым датчиком расстояния HC-SR04: http://iarduino.ru/shop/Sensory-Datchiki/ultrazvukovogo-datchika-hc-sr04-rasstoyaniya-dvizheniya.html
//  Версия: 1.0.0 (с использованием прерываний)
//  Последнюю версию библиотеки Вы можете скачать по ссылке: http://iarduino.ru/file/283.html
//  Подробное описание функции бибилиотеки доступно по ссылке: http://wiki.iarduino.ru/page/ultrazvukovoy-datchik-izmereniya-rasstoyaniya-hc-sr04/
//  Библиотека является собственностью интернет магазина iarduino.ru и может свободно использоваться и распространяться!
//  При публикации устройств или скетчей с использованием данной библиотеки, как целиком, так и её частей,
//  в том числе и в некоммерческих целях, просим Вас опубликовать ссылку: http://iarduino.ru
//  Автор библиотеки: Панькин Павел sajaem@narod.ru
//  Если у Вас возникли технические вопросы, напишите нам: shop@iarduino.ru

#ifndef iarduino_HC_SR04_int_h
#define iarduino_HC_SR04_int_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

class	iarduino_HC_SR04_int{
	public:			iarduino_HC_SR04_int(uint8_t, uint8_t);	//	инициализация модуля	(вывод TRIG, вывод ECHO)
		long		distance			(int8_t=23);		//	определение расстояния	([t°C])
	private:
		uint8_t		HC_pins_TRIG;							//	№ вывода подключённого к TRIG
		uint8_t		HC_pins_ECHO;							//	№ вывода подключённого к ECHO
		uint8_t		HC_ints_IRQ;							//	№ прерывания для вывода  ECHO
		long		HC_data_ANSWER;							//	переменная для рассчета ответа
};

class	iarduino_HC_SR04_int_volatile_class{
	public:
		bool		HC_flag_REQUEST[10];					//	флаг наличия запроса данных
		bool		HC_flag_ANSWER[10];						//	флаг начала импульса ответа
		uint32_t	HC_time_REQUEST[10];					//	время отправки запроса данных
		uint32_t	HC_time_ANSWER[10];						//	время прошедшее с момента запроса до получения данных
};


#endif
