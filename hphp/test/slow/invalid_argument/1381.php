<?php

// disable array -> "Array" conversion notice
error_reporting(error_reporting() & ~E_NOTICE);

$ch = curl_init();
var_dump(curl_setopt($ch, -1337, 'http://www.example.com/'));
curl_close($ch);
var_dump(iconv_set_encoding('internal_encoding',  str_pad('invalid-charset', 64)));
var_dump(iconv_mime_decode(  'Subject: =?UTF-8?B?UHLDvGZ1bmcgUHLDvGZ1bmc=?=',  0, str_pad('invalid-charset', 64)));
var_dump(iconv_mime_decode_headers( 'Subject: =?UTF-8?B?UHLDvGZ1bmcgUHLDvGZ1bmc=?=', 0, str_pad('invalid-charset', 64)));
var_dump(iconv_strlen('UHLDvGZ1bmcgUHLDvGZ1bmc=',  str_pad('invalid-charset', 64)));
$subject = 'Subject: =?UTF-8?B?UHLDvGZ1bmcgUHLDvGZ1bmc=?=';
var_dump(iconv_strpos($subject, 'H', 0, str_pad('invalid-charset', 64)));
var_dump(iconv_strrpos($subject, 'H', str_pad('invalid-charset', 64)));
var_dump(iconv_substr('AB',0,1, str_pad('invalid-charset', 64)));
$preferences = array(  'output-charset' => 'UTF-8',  'line-length' => 76,  'line-break-chars' => "\n");
$preferences['scheme'] = 'Q';
$preferences['input-charset'] = str_pad('invalid-charset', 64);
var_dump(iconv_mime_encode('Subject', "Pr\xc3\xbcfung Pr\xc3\xbcffung",  $preferences));
$preferences['input-charset'] = 'ISO-8859-1';
$preferences['output-charset'] = str_pad('invalid-charset', 64);
var_dump(iconv_mime_encode('Subject', "Pr\xc3\xbcfung Pr\xc3\xbcffung",  $preferences));
var_dump(iconv_set_encoding('internal_encoding', str_pad('invalid-charset', 64)));
var_dump(iconv('UTF-8', str_pad('invalid-charset', 64), ''));
var_dump(iconv(str_pad('invalid-charset', 64), 'UTF-8', ''));
var_dump(time_nanosleep(-1, 0));
var_dump(time_nanosleep(0, -1));
var_dump(time_sleep_until(0));
var_dump(gzcompress('abc', -2));
var_dump(gzdeflate('abc', -2));
var_dump(http_build_query(1));
var_dump(parse_url('http://www.example.com', 100));
var_dump(checkdnsrr('127.0.0.1', 'INVALID_TYPE'));
var_dump(assert_options(-1));
var_dump(simplexml_load_string('', 'INVALID_CLASS'));
var_dump(simplexml_load_string('', 'stdClass'));
var_dump(stream_get_contents('', -1));
$fp = fopen(__DIR__.'/../../sample_dir/file', 'r');
var_dump(fgets($fp, -1));
fclose($fp);
$tmpfname = tempnam('', str_repeat('a', 128));
var_dump(strlen(basename($tmpfname)));
unlink($tmpfname);
$tmpfname = tempnam('', '/var/www' . str_repeat('a', 128));
var_dump(strlen(basename($tmpfname)));
unlink($tmpfname);
$ar1 = array(10, 100, 100, 0);
$ar2 = array(1, 3, 2);
var_dump(array_multisort($ar1, $ar2));
$phrase  = 'eat fruits, vegetables, and fiber every day.';
$healthy = array('fruits', 'vegetables');
$yummy   = array('pizza', 'beer', 'ice cream');
var_dump(str_replace($healthy, $yummy, $phrase));
var_dump(str_replace('ll', $yummy, 'good golly miss molly!', $count));
var_dump(setlocale(LC_ALL, array('de_DE@garbage', 'de_DE', 'deu_deu'), array(1, 2)));
var_dump(setlocale(LC_ALL, str_pad('a', 255)));
var_dump(pack("\xf4", 0x1234, 0x5678, 65, 66));
var_dump(pack("x5", 0x1234, 0x5678, 65, 66));
var_dump(pack("h", -0x1234));
var_dump(pack("h", 12345678900));
var_dump(unpack("\xf4", "0x1234"));
var_dump(sscanf('foo', '[%s', $id, $first, $last));
var_dump(sscanf('foo', '%z', $id, $first, $last));
var_dump(sscanf("SN/abc", "SN/%d%d", $out));
var_dump($out);
var_dump(sscanf("SN/abc", "", $out));
var_dump($out);
var_dump(printf('%$', 3));
var_dump(vsprintf('%$', 3));
var_dump(sprintf('%$', 3));
var_dump(vsprintf('%$', 3));
var_dump(str_word_count('abc', 2, '...'));
var_dump(str_word_count('abc', 2, 'b..a'));
var_dump(str_word_count('abc', 2, 'a..b..c'));
var_dump(base_convert('05678', 8, 37));
var_dump(convert_cyr_string('abc', 'y', 'z'));
var_dump(money_format('%abc', 1.33));
var_dump(money_format('%i%i', 1.33));
var_dump(str_pad('abc', 10, '', 100));
var_dump(str_pad('abc', 10, ' ', 100));
var_dump(wordwrap('', 75, '', true));
var_dump(wordwrap(null, 75, '', true));
var_dump(wordwrap('abc', 75, '', true));
var_dump(wordwrap('abc', 0, '', true));
