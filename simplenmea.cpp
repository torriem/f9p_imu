#include "simplenmea.h"

//TODO: add url to original project, original copyright, LPGL license

// Allow debugging/regression testing under normal g++ environment.
#ifdef MICRONMEA_DEBUG
#include <stdlib.h>
#include <iostream>
using namespace std;
#endif


static long exp10(uint8_t b)
{
	long r = 1;
	while (b--)
		r *= 10;
	return r;
}


static char toHex(uint8_t nibble)
{
	if (nibble >= 10)
		return nibble + 'A' - 10;
	else
		return nibble + '0';

}


const char* SimpleNMEAParser::skipField(const char* s)
{
	if (s == NULL)
		return NULL;

	while (!isEndOfFields(*s)) {
		if (*s == ',') {
			// Check next character
			if (isEndOfFields(*++s))
				break;
			else
				return s;
		}
		++s;
	}
	return NULL; // End of string or valid sentence
}


unsigned int SimpleNMEAParser::parseUnsignedInt(const char *s, uint8_t len)
{
	int r = 0;
	while (len--)
		r = 10 * r + *s++ - '0';
	return r;
}


long SimpleNMEAParser::parseFloat(const char* s, uint8_t log10Multiplier, const char** eptr)
{
	int8_t neg = 1;
	long r = 0;
	while (isspace(*s))
		++s;
	if (*s == '-') {
		neg = -1;
		++s;
	}
	else if (*s == '+')
		++s;

	while (isdigit(*s))
		r = 10*r + *s++ - '0';
	r *= exp10(log10Multiplier);

	if (*s == '.') {
		++s;
		long frac = 0;
		while (isdigit(*s) && log10Multiplier) {
			frac = 10 * frac + *s++ -'0';
			--log10Multiplier;
		}
		frac *= exp10(log10Multiplier);
		r += frac;
	}
	r *= neg; // Include effect of any minus sign

	if (eptr)
		*eptr = skipField(s);

	return r;
}


long SimpleNMEAParser::parseDegreeMinute(const char* s, uint8_t degWidth,
								  const char **eptr)
{
	if (*s == ',') {
		if (eptr)
			*eptr = skipField(s);
		return 0;
	}
	long r = parseUnsignedInt(s, degWidth) * 10000000L;
	s += degWidth;
	r += parseFloat(s, 7, eptr) / 60;
	return r;
}


const char* SimpleNMEAParser::parseField(const char* s, char *result, int len)
{
	if (s == NULL)
		return NULL;

	int i = 0;
	while (*s != ',' && !isEndOfFields(*s)) {
		if (result && i++ < len)
			*result++ = *s;
		++s;
	}
	if (result && i < len)
		*result = '\0'; // Terminate unless too long

	if (*s == ',')
		return ++s; // Location of start of next field
	else
		return NULL; // End of string or valid sentence
}


const char* SimpleNMEAParser::generateChecksum(const char* s, char* checksum)
{
	uint8_t c = 0;
	// Initial $ is omitted from checksum, if present ignore it.
	if (*s == '$')
		++s;

	while (*s != '\0' && *s != '*')
		c ^= *s++;

	if (checksum) {
		checksum[0] = toHex(c / 16);
		checksum[1] = toHex(c % 16);
	}
	return s;
}


bool SimpleNMEAParser::testChecksum(const char* s)
{
	char checksum[2];
	const char* p = generateChecksum(s, checksum);
	return *p == '*' && p[1] == checksum[0] && p[2] == checksum[1];
}


#ifndef MICRONMEA_DEBUG
// When debugging in normal g++ environment ostream doesn't have a
// print member function. As sendSentence() isn't needed when
// debugging don't compile it.
Stream& SimpleNMEAParser::sendSentence(Stream& s, const char* sentence)
{
	char checksum[3];
	generateChecksum(sentence, checksum);
	checksum[2] = '\0';
	s.print(sentence);
	s.print('*');
	s.print(checksum);
	s.print("\r\n");
	return s;
}
#endif


SimpleNMEAParser::SimpleNMEAParser(void) :
	_badChecksumHandler(NULL),
	_unknownSentenceHandler(NULL)
{
	setBuffer(NULL, 0);
	clear();
}


SimpleNMEAParser::SimpleNMEAParser(void* buf, uint8_t len) :
	_badChecksumHandler(NULL),
	_unknownSentenceHandler(NULL)
{
	setBuffer(buf, len);
	clear();
}


void SimpleNMEAParser::setBuffer(void* buf, uint8_t len)
{
	_bufferLen = len;
	_buffer = (char*)buf;
	_ptr = _buffer;
	if (_bufferLen) {
		*_ptr = '\0';
		_buffer[_bufferLen - 1] = '\0';
	}
}


void SimpleNMEAParser::clear(void)
{
	_navSystem = '\0';
	_numSat = 0;
	_hdop = 255;
	_isValid = false;
	_latitude = 999000000L;
	_longitude = 999000000L;
	_altitude = _speed = _course = LONG_MIN;
	_altitudeValid = false;
	_year = _month = _day = 0;
	_hour = _minute = _second = 99;
	_hundredths = 0;
}


bool SimpleNMEAParser::process(char c)
{
	if (_buffer == NULL || _bufferLen == 0)
		return false;
	if (c == '\0' || c == '\n' || c == '\r') {
		// Terminate buffer then reset pointer
		*_ptr = '\0';
		sentence_length = _ptr - _buffer;
		_ptr = _buffer;

		if (*_buffer == '$' && testChecksum(_buffer)) {
			// Valid message
			const char* data;
			if (_buffer[1] == 'G') {
				_talkerID = _buffer[2];
				data = parseField(&_buffer[3], &_messageID[0], sizeof(_messageID));
			}
			else {
				_talkerID = '\0';
				data = parseField(&_buffer[1], &_messageID[0], sizeof(_messageID));
			}

			if (strcmp(&_messageID[0], "GGA") == 0)
				return processGGA(data);
			else if (strcmp(&_messageID[0], "VTG") == 0)
				return processVTG(data);
			else if (_unknownSentenceHandler)
				(*_unknownSentenceHandler)(*this);
		}
		else {
			if (_badChecksumHandler && *_buffer != '\0') // don't send empty buffers as bad checksums!
				(*_badChecksumHandler)(*this);
		}
		// Return true for a complete, non-empty, sentence (even if not a valid one).
		return *_buffer != '\0'; //
	}
	else {
		*_ptr = c;
		if (_ptr < &_buffer[_bufferLen - 1])
			++_ptr;
	}

	return false;
}


const char* SimpleNMEAParser::parseTime(const char* s)
{
	if (*s == ',')
		return skipField(s);
	_hour = parseUnsignedInt(s, 2);
	_minute = parseUnsignedInt(s + 2, 2);
	_second = parseUnsignedInt(s + 4, 2);
	_hundredths = parseUnsignedInt(s + 7, 2);
	return skipField(s + 9);
}


const char* SimpleNMEAParser::parseDate(const char* s)
{
	if (*s == ',')
		return skipField(s);
	_day = parseUnsignedInt(s, 2);
	_month = parseUnsignedInt(s + 2, 2);
	_year = parseUnsignedInt(s + 4, 2) + 2000;
	return skipField(s + 6);
}


bool SimpleNMEAParser::processGGA(const char *s)
{
	// If GxGSV messages are received _talker_ID can be changed after
	// other MicroNMEA sentences. Compatibility modes can set the talker ID
	// to indicate GPS regardless of actual navigation system used.
	_navSystem = _talkerID;

	s = skipField(s);
	//s = parseTime(s);
	// ++s;
	
	before_latlon = s - _buffer;
	_latitude = parseDegreeMinute(s, 2, &s);
	if (*s == ',')
		++s;
	else {
		if (*s == 'S')
			_latitude *= -1;
		s += 2; // Skip N/S and comma
	}
	_longitude = parseDegreeMinute(s, 3, &s);
	if (*s == ',')
		++s;
	else {
		if (*s == 'W')
			_longitude *= -1;
		s += 2; // Skip E/W and comma
	}

	after_latlon = s - _buffer;
	
	_isValid = (*s != '0');
	_fixType = parseFloat(s,0, &s);

	//_numSat = parseFloat(s, 0, &s);
	s = skipField(s);
	//_hdop = parseFloat(s, 1, &s);
	s = skipField(s);

	before_altitude = s - _buffer;
	_altitude = parseFloat(s, 3, &s);
	_altitudeValid = true;
	afer_altitude = s - _buffer;
	// That's all we care about
	
	return true;
}

bool SimpleNMEAParser::processVTG(const char *s)
{
	_navSystem = _talkerID;

	_course = parseFloat(s, 1, &s);
	s = skipField(s); //should be 'T'
	s = skipField(s); //magnetic heading
	s = skipField(s); //might be 'M'
	s = skipField(s); //speed in knots
	s = skipField(s); //usually 'N'
	_speed = parseFloat(s,1,&s); //speed in km/h
	s = skipField(s); //usually 'K'
	//that's all we care about.
	
	return true;
}

//bool GGNMEA::processRMC(const char *s)
//{
//	_navSystem = _talkerID;
//}
