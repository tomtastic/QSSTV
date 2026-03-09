/******************************************************************************\
 * Technische Universitaet Darmstadt, Institut fuer Nachrichtentechnik
 * Copyright (c) 2001
 *
 * Author(s):
 *	Volker Fischer
 *
 * Adapted for ham sstv use Ties Bos - PA0MBO
 *
 * Description:
 *	Tables for FAC
 *
 ******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
\******************************************************************************/

#include "TableFAC.h"

/* Definitions ****************************************************************/
/* ETSI ES201980V2.1.1: page 115, 7.5.3: ...FAC shall use 4-QAM mapping. A
   fixed code rate shall be applied...R_all=0.6...
   6 tailbits are used for the encoder to get in zero state ->
   65 [number of cells] * 2 [4-QAM] * 0.6 [code-rate] - 6 [tailbits] = 72 */
// #define NUM_FAC_BITS_PER_BLOCK			72

/* iTableNumOfServices[a][b]
   a: Number of audio services
   b: Number of data services
   (6.3.4) */
const int iTableNumOfServices[5][5] = {
    /* -> Data */
    {-1, 1, 2, 3, 15},
    {4, 5, 6, 7, -1},
    {8, 9, 10, -1, -1},
    {12, 13, -1, -1, -1},
    {0, -1, -1, -1, -1}};

/* Language code */
#define LEN_TABLE_LANGUAGE_CODE 16

const string strTableLanguageCode[LEN_TABLE_LANGUAGE_CODE] = {"No language specified",
                                                              "Arabic",
                                                              "Bengali",
                                                              "Chinese (Mandarin)",
                                                              "Dutch",
                                                              "English",
                                                              "French",
                                                              "German",
                                                              "Hindi",
                                                              "Japanese",
                                                              "Javanese",
                                                              "Korean",
                                                              "Portuguese",
                                                              "Russian",
                                                              "Spanish",
                                                              "Other language"};

/* Programme Type codes */
#define LEN_TABLE_PROG_TYPE_CODE_TOT 32
#define LEN_TABLE_PROG_TYPE_CODE 30

const string strTableProgTypCod[LEN_TABLE_PROG_TYPE_CODE_TOT] = {"No programme type",
                                                                 "News",
                                                                 "Current Affairs",
                                                                 "Information",
                                                                 "Sport",
                                                                 "Education",
                                                                 "Drama",
                                                                 "Culture",
                                                                 "Science",
                                                                 "Varied",
                                                                 "Pop Music",
                                                                 "Rock Music",
                                                                 "Easy Listening Music",
                                                                 "Light Classical",
                                                                 "Serious Classical",
                                                                 "Other Music",
                                                                 "Weather/meteorology",
                                                                 "Finance/Business",
                                                                 "Children's programmes",
                                                                 "Social Affairs",
                                                                 "Religion",
                                                                 "Phone In",
                                                                 "Travel",
                                                                 "Leisure",
                                                                 "Jazz Music",
                                                                 "Country Music",
                                                                 "National Music",
                                                                 "Oldies Music",
                                                                 "Folk Music",
                                                                 "Documentary",
                                                                 "Not used",
                                                                 "Not used"};


/* Country code table according to ISO 3166 */

// const struct elCountry TableCountryCode[LEN_TABLE_COUNTRY_CODE] = {
//	{"af", "Afghanistan"},
//	{"ax", "Aland Islands"},
//	{"al", "Albania"},
//	{"dz", "Algeria"},
//	{"as", "American Samoa"},
//	{"ad", "Andorra"},
//	{"ao", "Angola"},
//	{"ai", "Anguilla"},
//	{"aq", "Antarctica"},
//	{"ag", "Antigua and barbuda"},
//	{"ar", "Argentina"},
//	{"am", "Armenia"},
//	{"aw", "Aruba"},
//	{"au", "Australia"},
//	{"at", "Austria"},
//	{"az", "Azerbaijan"},
//	{"bs", "Bahamas"},
//	{"bh", "Bahrain"},
//	{"bd", "Bangladesh"},
//	{"bb", "Barbados"},
//	{"by", "Belarus"},
//	{"be", "Belgium"},
//	{"bz", "Belize"},
//	{"bj", "Benin"},
//	{"bm", "Bermuda"},
//	{"bt", "Bhutan"},
//	{"bo", "Bolivia"},
//	{"ba", "Bosnia and Herzegovina"},
//	{"bw", "Botswana"},
//	{"bv", "Bouvet Island"},
//	{"br", "Brazil"},
//	{"io", "British Indian Ocean Ter."},
//	{"bn", "Brunei Darussalam"},
//	{"bg", "Bulgaria"},
//	{"bf", "Burkina Faso"},
//	{"bi", "Burundi"},
//	{"kh", "Cambodia"},
//	{"cm", "Cameroon"},
//	{"ca", "Canada"},
//	{"cv", "Cape Verde"},
//	{"ky", "Cayman Islands"},
//	{"cf", "Central African Republic"},
//	{"td", "Chad"},
//	{"cl", "Chile"},
//	{"cn", "China"},
//	{"cx", "Christmas Island"},
//	{"cc", "Cocos (Keeling) Islands"},
//	{"co", "Colombia"},
//	{"km", "Comoros"},
//	{"cg", "Congo Democratic Rep."},
//	{"cd", "Congo"},
//	{"ck", "Cook Islands"},
//	{"cr", "Costa Rica"},
