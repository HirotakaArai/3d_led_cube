#pragma once
class bitmap
{
private:

public:
	
	bitmap();
	~bitmap();

	unsigned int height;
	unsigned int width;
	unsigned char *Rdata;
	unsigned char *Gdata;
	unsigned char *Bdata;

	//�擾�ɐ�������΃|�C���^�����s�����NULL��Ԃ�
	bool read_bmp(char* filename);
};

