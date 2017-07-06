#pragma once

#define LED_WIDTH	16
#define LED_HEIGHT 32
#define LED_DEPTH	8

/*!
3D LED�𐧌䂵�Ă���Raspberry Pi��URL�iIP�A�h���X�j���w�肷��
�{���\�b�h���R�[�����邱�ƂŁAShow�̂��т�Raspberry Pi�փf�[�^���]�������B
@param[in] url Raspberry Pi��URL�iIP�A�h���X�j
*/
typedef void * SetUrl_t(char const * url);

/*!
LED�P�̂̐F��ݒ肷��
�{�֐��͐F�̐ݒ���L���b�V�����邾���ŕ\���̍X�V�͂��Ȃ��B
@param[in] x X�������W�i0 <= x < LED_WIDTH�j
@param[in] y Y�������W�i0 <= y < LED_HEIGHT�j
@param[in] z Z�������W�i0 <= z < LED_DEPTH�j
@param[in] rgb �F�i0x00000000 <= rgb <= 0x00FFFFFF�j
@note �͈͊O�̃p�����[�^���w�肵���ꍇ�̓���͖���`
*/
typedef void * SetLed_t(int x, int y, int z, int rgb);

/*!
LED�ݒ�̃L���b�V�����N���A����
*/
typedef void * Clear_t();

/*!
SetLed�Őݒ肵���Ƃ����GUI��3D LED��\������
SetUrl�Ăяo����́ARaspberryPi�֕`�����]������
*/
typedef void * Show_t();


/*!
�E�F�C�g����
GUI��LED��\������ɂ́A�{�֐��̃R�[�����K�v
Show�֐���Raspberry Pi�ւ̓]�����鎞�Ԃ��l�������E�F�C�g������
@param[in] ms �ҋ@����[�~���b]
*/
typedef void * Wait_t(int ms);
