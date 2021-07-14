
#pragma once

#include <stdio.h>

#define DefaultYPelsPerMeter 3780
#define DefaultXPelsPerMeter 3780

bool IsBigEndian()
{
	short word = 0x0001;
	if ((*(char*)&word) != 0x01)
	{
		return true;
	}
	return false;
}

inline unsigned short FlipWORD(unsigned short in)
{
	return ((in >> 8) | (in << 8));
}

inline unsigned int FlipDWORD(unsigned int in)
{
	return (((in & 0xFF000000) >> 24) | ((in & 0x000000FF) << 24) |
		((in & 0x00FF0000) >> 8) | ((in & 0x0000FF00) << 8));
}

typedef struct RGBApixel {
	RGBApixel() {}
	RGBApixel(float s)
	{
		Blue = (unsigned char)(255 * s);
		Green = (unsigned char)(255 * s);
		Red = (unsigned char)(255 * s);
		Alpha = 255;
	}
	RGBApixel(unsigned char b, unsigned char g, unsigned char r)
	{
		Blue = b;
		Green = g;
		Red = r;
		Alpha = 255;
	}
	unsigned char Blue;
	unsigned char Green;
	unsigned char Red;
	unsigned char Alpha;
} RGBApixel;

class BMFH {
public:
	unsigned short  bfType;
	unsigned int bfSize;
	unsigned short  bfReserved1;
	unsigned short  bfReserved2;
	unsigned int bfOffBits;

	BMFH()
	{
		bfType = 19778;
		bfReserved1 = 0;
		bfReserved2 = 0;
	}
	void SwitchEndianess()
	{
		bfType = FlipWORD(bfType);
		bfSize = FlipDWORD(bfSize);
		bfReserved1 = FlipWORD(bfReserved1);
		bfReserved2 = FlipWORD(bfReserved2);
		bfOffBits = FlipDWORD(bfOffBits);
	}
};

class BMIH {
public:
	unsigned int biSize;
	unsigned int biWidth;
	unsigned int biHeight;
	unsigned short  biPlanes;
	unsigned short  biBitCount;
	unsigned int biCompression;
	unsigned int biSizeImage;
	unsigned int biXPelsPerMeter;
	unsigned int biYPelsPerMeter;
	unsigned int biClrUsed;
	unsigned int biClrImportant;

	BMIH()
	{
		biPlanes = 1;
		biCompression = 0;
		biXPelsPerMeter = DefaultXPelsPerMeter;
		biYPelsPerMeter = DefaultYPelsPerMeter;
		biClrUsed = 0;
		biClrImportant = 0;
	}
	void SwitchEndianess()
	{
		biSize = FlipDWORD(biSize);
		biWidth = FlipDWORD(biWidth);
		biHeight = FlipDWORD(biHeight);
		biPlanes = FlipWORD(biPlanes);
		biBitCount = FlipWORD(biBitCount);
		biCompression = FlipDWORD(biCompression);
		biSizeImage = FlipDWORD(biSizeImage);
		biXPelsPerMeter = FlipDWORD(biXPelsPerMeter);
		biYPelsPerMeter = FlipDWORD(biYPelsPerMeter);
		biClrUsed = FlipDWORD(biClrUsed);
		biClrImportant = FlipDWORD(biClrImportant);
	}
};

class BMPFile
{
public:


	BMPFile()
	{
		Width = 1;
		Height = 1;
		BitDepth = 24;
		Pixels = new RGBApixel * [Width];
		Pixels[0] = new RGBApixel[Height];
		Colors = nullptr;

		XPelsPerMeter = 0;
		YPelsPerMeter = 0;

		MetaData1 = nullptr;
		SizeOfMetaData1 = 0;
		MetaData2 = nullptr;
		SizeOfMetaData2 = 0;
	}

	BMPFile(BMPFile& Input)
	{
		Width = 1;
		Height = 1;
		BitDepth = 24;
		Pixels = new RGBApixel * [Width];
		Pixels[0] = new RGBApixel[Height];
		Colors = nullptr;
		XPelsPerMeter = 0;
		YPelsPerMeter = 0;

		MetaData1 = nullptr;
		SizeOfMetaData1 = 0;
		MetaData2 = nullptr;
		SizeOfMetaData2 = 0;

		SetBitDepth(Input.GetBitDepth());
		SetSize(Input.GetWidth(), Input.GetHeight());
		SetDPI(Input.GetHorizontalDPI(), Input.GetVerticalDPI());

		if (BitDepth == 1 || BitDepth == 4 || BitDepth == 8)
		{
			for (int k = 0; k < GetNumberOfColors(); k++)
			{
				SetColor(k, Input.GetColor(k));
			}
		}

		for (int j = 0; j < Height; j++)
		{
			for (int i = 0; i < Width; i++)
			{
				Pixels[i][j] = *Input(i, j);
			}
		}
	}

	~BMPFile()
	{
		int i;
		for (i = 0; i < Width; i++)
		{
			delete[] Pixels[i];
		}
		delete[] Pixels;
		if (Colors)
		{
			delete[] Colors;
		}

		if (MetaData1)
		{
			delete[] MetaData1;
		}
		if (MetaData2)
		{
			delete[] MetaData2;
		}
	}

	RGBApixel* operator()(int i, int j)
	{
		if (i >= Width)
		{
			i = Width - 1;
		}
		if (i < 0)
		{
			i = 0;
		}
		if (j >= Height)
		{
			j = Height - 1;
		}
		if (j < 0)
		{
			j = 0;
		}
		return &Pixels[i][j];
	}

	int GetBitDepth() const
	{
		return BitDepth;
	}
	int GetWidth() const
	{
		return Width;
	}
	int GetHeight() const
	{
		return Height;
	}

	int GetNumberOfColors()
	{
		int output = IntPow(2, BitDepth);
		if (BitDepth == 32)
		{
			output = IntPow(2, 24);
		}
		return output;
	}

	void SetDPI(int HorizontalDPI, int VerticalDPI)
	{
		XPelsPerMeter = (int)(HorizontalDPI * 39.37007874015748);
		YPelsPerMeter = (int)(VerticalDPI * 39.37007874015748);
	}

	int GetVerticalDPI()
	{
		if (!YPelsPerMeter)
		{
			YPelsPerMeter = DefaultYPelsPerMeter;
		}
		return (int)(YPelsPerMeter / (double)39.37007874015748);
	}

	int GetHorizontalDPI()
	{
		if (!XPelsPerMeter)
		{
			XPelsPerMeter = DefaultXPelsPerMeter;
		}
		return (int)(XPelsPerMeter / (double)39.37007874015748);
	}

	void LoadBitmap(int* pPixel, int _Width, int _Height, float scale)
	{
		SetSize(_Width, _Height);
		for (int i = 0; i < _Width; ++i)
		for (int j = 0; j < _Height; ++j)
		{
			SetPixel(i, j, RGBApixel(scale * pPixel[j * _Width + i]));
		}
	}

	void LoadBitmap(float* pPixel, int _Width, int _Height, float scale)
	{
		SetSize(_Width, _Height);
		for (int i = 0; i < _Width; ++i)
		for (int j = 0; j < _Height; ++j)
		{
			SetPixel(i, j, RGBApixel(scale * pPixel[j * _Width + i]));
		}
	}

	const RGBApixel& GetPixel(int i, int j) const
	{
		if (i >= Width)
		{
			i = Width - 1;
		}
		if (i < 0)
		{
			i = 0;
		}
		if (j >= Height)
		{
			j = Height - 1;
		}
		if (j < 0)
		{
			j = 0;
		}
		return Pixels[i][j];
	}

	void SetPixel(int i, int j, RGBApixel NewPixel)
	{
		Pixels[i][j] = NewPixel;
	}

	void SetPixel(int i, int j, unsigned char g, unsigned char b, unsigned char r)
	{
		Pixels[i][j] = RGBApixel(g, b, r);
	}

	bool SetSize(int NewWidth, int NewHeight)
	{
		if (NewWidth <= 0 || NewHeight <= 0)
		{
			return false;
		}

		int i, j;

		for (i = 0; i < Width; i++)
		{
			delete[] Pixels[i];
		}
		delete[] Pixels;

		Width = NewWidth;
		Height = NewHeight;
		Pixels = new RGBApixel * [Width];

		for (i = 0; i < Width; i++)
		{
			Pixels[i] = new RGBApixel[Height];
		}

		for (i = 0; i < Width; i++)
		{
			for (j = 0; j < Height; j++)
			{
				Pixels[i][j].Red = 255;
				Pixels[i][j].Green = 255;
				Pixels[i][j].Blue = 255;
				Pixels[i][j].Alpha = 0;
			}
		}

		return true;
	}

	bool SetBitDepth(int NewDepth)
	{
		using namespace std;
		if (NewDepth != 1 && NewDepth != 4 &&
			NewDepth != 8 && NewDepth != 16 &&
			NewDepth != 24 && NewDepth != 32)
		{
			return false;
		}

		BitDepth = NewDepth;
		if (Colors)
		{
			delete[] Colors;
		}
		int NumberOfColors = IntPow(2, BitDepth);
		if (BitDepth == 1 || BitDepth == 4 || BitDepth == 8)
		{
			Colors = new RGBApixel[NumberOfColors];
		}
		else
		{
			Colors = nullptr;
		}
		if (BitDepth == 1 || BitDepth == 4 || BitDepth == 8)
		{
			CreateStandardColorTable();
		}

		return true;
	}

	bool WriteToFile(const char* FileName)
	{
		FILE* fp = fopen(FileName, "wb");
		if (fp == nullptr)
		{
			fclose(fp);
			return false;
		}

		// some preliminaries

		double dBytesPerPixel = ((double)BitDepth) / 8.0;
		double dBytesPerRow = dBytesPerPixel * (Width + 0.0);
		dBytesPerRow = ceil(dBytesPerRow);

		int BytePaddingPerRow = 4 - ((int)(dBytesPerRow)) % 4;
		if (BytePaddingPerRow == 4)
		{
			BytePaddingPerRow = 0;
		}

		double dActualBytesPerRow = dBytesPerRow + BytePaddingPerRow;

		double dTotalPixelBytes = Height * dActualBytesPerRow;

		double dPaletteSize = 0;
		if (BitDepth == 1 || BitDepth == 4 || BitDepth == 8)
		{
			dPaletteSize = IntPow(2, BitDepth) * 4.0;
		}

		// leave some room for 16-bit masks 
		if (BitDepth == 16)
		{
			dPaletteSize = 3 * 4;
		}

		double dTotalFileSize = 14 + 40 + dPaletteSize + dTotalPixelBytes;

		// write the file header 

		BMFH bmfh;
		bmfh.bfSize = (unsigned int)dTotalFileSize;
		bmfh.bfReserved1 = 0;
		bmfh.bfReserved2 = 0;
		bmfh.bfOffBits = (unsigned int)(14 + 40 + dPaletteSize);

		if (IsBigEndian())
		{
			bmfh.SwitchEndianess();
		}

		fwrite((char*)&(bmfh.bfType), sizeof(unsigned short), 1, fp);
		fwrite((char*)&(bmfh.bfSize), sizeof(unsigned int), 1, fp);
		fwrite((char*)&(bmfh.bfReserved1), sizeof(unsigned short), 1, fp);
		fwrite((char*)&(bmfh.bfReserved2), sizeof(unsigned short), 1, fp);
		fwrite((char*)&(bmfh.bfOffBits), sizeof(unsigned int), 1, fp);

		// write the info header 

		BMIH bmih;
		bmih.biSize = 40;
		bmih.biWidth = Width;
		bmih.biHeight = Height;
		bmih.biPlanes = 1;
		bmih.biBitCount = BitDepth;
		bmih.biCompression = 0;
		bmih.biSizeImage = (unsigned int)dTotalPixelBytes;
		if (XPelsPerMeter)
		{
			bmih.biXPelsPerMeter = XPelsPerMeter;
		}
		else
		{
			bmih.biXPelsPerMeter = DefaultXPelsPerMeter;
		}
		if (YPelsPerMeter)
		{
			bmih.biYPelsPerMeter = YPelsPerMeter;
		}
		else
		{
			bmih.biYPelsPerMeter = DefaultYPelsPerMeter;
		}

		bmih.biClrUsed = 0;
		bmih.biClrImportant = 0;

		// indicates that we'll be using bit fields for 16-bit files
		if (BitDepth == 16)
		{
			bmih.biCompression = 3;
		}

		if (IsBigEndian())
		{
			bmih.SwitchEndianess();
		}

		fwrite((char*)&(bmih.biSize), sizeof(unsigned int), 1, fp);
		fwrite((char*)&(bmih.biWidth), sizeof(unsigned int), 1, fp);
		fwrite((char*)&(bmih.biHeight), sizeof(unsigned int), 1, fp);
		fwrite((char*)&(bmih.biPlanes), sizeof(unsigned short), 1, fp);
		fwrite((char*)&(bmih.biBitCount), sizeof(unsigned short), 1, fp);
		fwrite((char*)&(bmih.biCompression), sizeof(unsigned int), 1, fp);
		fwrite((char*)&(bmih.biSizeImage), sizeof(unsigned int), 1, fp);
		fwrite((char*)&(bmih.biXPelsPerMeter), sizeof(unsigned int), 1, fp);
		fwrite((char*)&(bmih.biYPelsPerMeter), sizeof(unsigned int), 1, fp);
		fwrite((char*)&(bmih.biClrUsed), sizeof(unsigned int), 1, fp);
		fwrite((char*)&(bmih.biClrImportant), sizeof(unsigned int), 1, fp);

		// write the palette 
		if (BitDepth == 1 || BitDepth == 4 || BitDepth == 8)
		{
			int NumberOfColors = IntPow(2, BitDepth);

			// if there is no palette, create one 
			if (!Colors)
			{
				if (!Colors)
				{
					Colors = new RGBApixel[NumberOfColors];
				}
				CreateStandardColorTable();
			}

			int n;
			for (n = 0; n < NumberOfColors; n++)
			{
				fwrite((char*)&(Colors[n]), 4, 1, fp);
			}
		}

		// write the pixels 
		int i, j;
		if (BitDepth != 16)
		{
			unsigned char* Buffer;
			int BufferSize = (int)((Width * BitDepth) / 8.0);
			while (8 * BufferSize < Width * BitDepth)
			{
				BufferSize++;
			}
			while (BufferSize % 4)
			{
				BufferSize++;
			}

			Buffer = new unsigned char[BufferSize];
			for (j = 0; j < BufferSize; j++)
			{
				Buffer[j] = 0;
			}

			j = Height - 1;

			while (j > -1)
			{
				bool Success = false;
				if (BitDepth == 32)
				{
					Success = Write32bitRow(Buffer, BufferSize, j);
				}
				if (BitDepth == 24)
				{
					Success = Write24bitRow(Buffer, BufferSize, j);
				}
				if (BitDepth == 8)
				{
					Success = Write8bitRow(Buffer, BufferSize, j);
				}
				if (BitDepth == 4)
				{
					Success = Write4bitRow(Buffer, BufferSize, j);
				}
				if (BitDepth == 1)
				{
					Success = Write1bitRow(Buffer, BufferSize, j);
				}
				if (Success)
				{
					int BytesWritten = (int)fwrite((char*)Buffer, 1, BufferSize, fp);
					if (BytesWritten != BufferSize)
					{
						Success = false;
					}
				}
				if (!Success)
				{
					j = -1;
				}
				j--;
			}

			delete[] Buffer;
		}

		if (BitDepth == 16)
		{
			// write the bit masks

			unsigned short BlueMask = 31;    // bits 12-16
			unsigned short GreenMask = 2016; // bits 6-11
			unsigned short RedMask = 63488;  // bits 1-5
			unsigned short ZeroWORD;

			if (IsBigEndian())
			{
				RedMask = FlipWORD(RedMask);
			}
			fwrite((char*)&RedMask, 2, 1, fp);
			fwrite((char*)&ZeroWORD, 2, 1, fp);

			if (IsBigEndian())
			{
				GreenMask = FlipWORD(GreenMask);
			}
			fwrite((char*)&GreenMask, 2, 1, fp);
			fwrite((char*)&ZeroWORD, 2, 1, fp);

			if (IsBigEndian())
			{
				BlueMask = FlipWORD(BlueMask);
			}
			fwrite((char*)&BlueMask, 2, 1, fp);
			fwrite((char*)&ZeroWORD, 2, 1, fp);

			int DataBytes = Width * 2;
			int PaddingBytes = (4 - DataBytes % 4) % 4;

			// write the actual pixels

			for (j = Height - 1; j >= 0; j--)
			{
				// write all row pixel data
				i = 0;
				int WriteNumber = 0;
				while (WriteNumber < DataBytes)
				{
					unsigned short TempWORD;

					unsigned short RedWORD = (unsigned short)((Pixels[i][j]).Red / 8);
					unsigned short GreenWORD = (unsigned short)((Pixels[i][j]).Green / 4);
					unsigned short BlueWORD = (unsigned short)((Pixels[i][j]).Blue / 8);

					TempWORD = (RedWORD << 11) + (GreenWORD << 5) + BlueWORD;
					if (IsBigEndian())
					{
						TempWORD = FlipWORD(TempWORD);
					}

					fwrite((char*)&TempWORD, 2, 1, fp);
					WriteNumber += 2;
					i++;
				}
				// write any necessary row padding
				WriteNumber = 0;
				while (WriteNumber < PaddingBytes)
				{
					unsigned char TempBYTE;
					fwrite((char*)&TempBYTE, 1, 1, fp);
					WriteNumber++;
				}
			}

		}

		fclose(fp);
		return true;
	}

	bool ReadFromFile(const char* FileName)
	{
		FILE* fp = fopen(FileName, "rb");
		if (fp == nullptr)
		{
			SetBitDepth(1);
			SetSize(1, 1);
			return false;
		}

		// read the file header 

		BMFH bmfh;
		bool NotCorrupted = true;

		NotCorrupted &= SafeFread((char*)&(bmfh.bfType), sizeof(unsigned short), 1, fp);

		bool IsBitmap = false;

		if (IsBigEndian() && bmfh.bfType == 16973)
		{
			IsBitmap = true;
		}
		if (!IsBigEndian() && bmfh.bfType == 19778)
		{
			IsBitmap = true;
		}

		if (!IsBitmap)
		{
			fclose(fp);
			return false;
		}

		NotCorrupted &= SafeFread((char*)&(bmfh.bfSize), sizeof(unsigned int), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmfh.bfReserved1), sizeof(unsigned short), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmfh.bfReserved2), sizeof(unsigned short), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmfh.bfOffBits), sizeof(unsigned int), 1, fp);

		if (IsBigEndian())
		{
			bmfh.SwitchEndianess();
		}

		// read the info header

		BMIH bmih;

		NotCorrupted &= SafeFread((char*)&(bmih.biSize), sizeof(unsigned int), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmih.biWidth), sizeof(unsigned int), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmih.biHeight), sizeof(unsigned int), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmih.biPlanes), sizeof(unsigned short), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmih.biBitCount), sizeof(unsigned short), 1, fp);

		NotCorrupted &= SafeFread((char*)&(bmih.biCompression), sizeof(unsigned int), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmih.biSizeImage), sizeof(unsigned int), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmih.biXPelsPerMeter), sizeof(unsigned int), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmih.biYPelsPerMeter), sizeof(unsigned int), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmih.biClrUsed), sizeof(unsigned int), 1, fp);
		NotCorrupted &= SafeFread((char*)&(bmih.biClrImportant), sizeof(unsigned int), 1, fp);

		if (IsBigEndian())
		{
			bmih.SwitchEndianess();
		}

		// a safety catch: if any of the header information didn't read properly, abort
		// future idea: check to see if at least most is self-consistent

		if (!NotCorrupted)
		{
			SetSize(1, 1);
			SetBitDepth(1);
			fclose(fp);
			return false;
		}

		XPelsPerMeter = bmih.biXPelsPerMeter;
		YPelsPerMeter = bmih.biYPelsPerMeter;

		// if bmih.biCompression 1 or 2, then the file is RLE compressed

		if (bmih.biCompression == 1 || bmih.biCompression == 2)
		{
			SetSize(1, 1);
			SetBitDepth(1);
			fclose(fp);
			return false;
		}

		// if bmih.biCompression > 3, then something strange is going on 
		// it's probably an OS2 bitmap file.

		if (bmih.biCompression > 3)
		{
			SetSize(1, 1);
			SetBitDepth(1);
			fclose(fp);
			return false;
		}

		if (bmih.biCompression == 3 && bmih.biBitCount != 16)
		{
			SetSize(1, 1);
			SetBitDepth(1);
			fclose(fp);
			return false;
		}

		// set the bit depth

		int TempBitDepth = (int)bmih.biBitCount;
		if (TempBitDepth != 1 && TempBitDepth != 4
			&& TempBitDepth != 8 && TempBitDepth != 16
			&& TempBitDepth != 24 && TempBitDepth != 32)
		{
			SetSize(1, 1);
			SetBitDepth(1);
			fclose(fp);
			return false;
		}
		SetBitDepth((int)bmih.biBitCount);

		// set the size

		if ((int)bmih.biWidth <= 0 || (int)bmih.biHeight <= 0)
		{
			SetSize(1, 1);
			SetBitDepth(1);
			fclose(fp);
			return false;
		}
		SetSize((int)bmih.biWidth, (int)bmih.biHeight);

		// some preliminaries

		double dBytesPerPixel = ((double)BitDepth) / 8.0;
		double dBytesPerRow = dBytesPerPixel * (Width + 0.0);
		dBytesPerRow = ceil(dBytesPerRow);

		int BytePaddingPerRow = 4 - ((int)(dBytesPerRow)) % 4;
		if (BytePaddingPerRow == 4)
		{
			BytePaddingPerRow = 0;
		}

		// if < 16 bits, read the palette

		if (BitDepth < 16)
		{
			// determine the number of colors specified in the 
			// color table

			int NumberOfColorsToRead = ((int)bmfh.bfOffBits - 54) / 4;
			if (NumberOfColorsToRead > IntPow(2, BitDepth))
			{
				NumberOfColorsToRead = IntPow(2, BitDepth);
			}

			if (NumberOfColorsToRead < GetNumberOfColors())
			{
			}

			int n;
			for (n = 0; n < NumberOfColorsToRead; n++)
			{
				SafeFread((char*)&(Colors[n]), 4, 1, fp);
			}
			for (n = NumberOfColorsToRead; n < GetNumberOfColors(); n++)
			{
				RGBApixel WHITE;
				WHITE.Red = 255;
				WHITE.Green = 255;
				WHITE.Blue = 255;
				WHITE.Alpha = 0;
				SetColor(n, WHITE);
			}


		}

		// skip blank data if bfOffBits so indicates

		int BytesToSkip = bmfh.bfOffBits - 54;;
		if (BitDepth < 16)
		{
			BytesToSkip -= 4 * IntPow(2, BitDepth);
		}
		if (BitDepth == 16 && bmih.biCompression == 3)
		{
			BytesToSkip -= 3 * 4;
		}
		if (BytesToSkip < 0)
		{
			BytesToSkip = 0;
		}
		if (BytesToSkip > 0 && BitDepth != 16)
		{
			unsigned char* TempSkipBYTE;
			TempSkipBYTE = new unsigned char[BytesToSkip];
			SafeFread((char*)TempSkipBYTE, BytesToSkip, 1, fp);
			delete[] TempSkipBYTE;
		}

		// This code reads 1, 4, 8, 24, and 32-bpp files 
		// with a more-efficient buffered technique.

		int i, j;
		if (BitDepth != 16)
		{
			int BufferSize = (int)((Width * BitDepth) / 8.0);
			while (8 * BufferSize < Width * BitDepth)
			{
				BufferSize++;
			}
			while (BufferSize % 4)
			{
				BufferSize++;
			}
			unsigned char* Buffer;
			Buffer = new unsigned char[BufferSize];
			j = Height - 1;
			while (j > -1)
			{
				int BytesRead = (int)fread((char*)Buffer, 1, BufferSize, fp);
				if (BytesRead < BufferSize)
				{
					j = -1;
				}
				else
				{
					bool Success = false;
					if (BitDepth == 1)
					{
						Success = Read1bitRow(Buffer, BufferSize, j);
					}
					if (BitDepth == 4)
					{
						Success = Read4bitRow(Buffer, BufferSize, j);
					}
					if (BitDepth == 8)
					{
						Success = Read8bitRow(Buffer, BufferSize, j);
					}
					if (BitDepth == 24)
					{
						Success = Read24bitRow(Buffer, BufferSize, j);
					}
					if (BitDepth == 32)
					{
						Success = Read32bitRow(Buffer, BufferSize, j);
					}
					if (!Success)
					{
						j = -1;
					}
				}
				j--;
			}
			delete[] Buffer;
		}

		if (BitDepth == 16)
		{
			int DataBytes = Width * 2;
			int PaddingBytes = (4 - DataBytes % 4) % 4;

			// set the default mask

			unsigned short BlueMask = 31; // bits 12-16
			unsigned short GreenMask = 992; // bits 7-11
			unsigned short RedMask = 31744; // bits 2-6

			// read the bit fields, if necessary, to 
			// override the default 5-5-5 mask

			if (bmih.biCompression != 0)
			{
				// read the three bit masks

				unsigned short TempMaskWORD;
				// unsigned short ZeroWORD;

				SafeFread((char*)&RedMask, 2, 1, fp);
				if (IsBigEndian())
				{
					RedMask = FlipWORD(RedMask);
				}
				SafeFread((char*)&TempMaskWORD, 2, 1, fp);

				SafeFread((char*)&GreenMask, 2, 1, fp);
				if (IsBigEndian())
				{
					GreenMask = FlipWORD(GreenMask);
				}
				SafeFread((char*)&TempMaskWORD, 2, 1, fp);

				SafeFread((char*)&BlueMask, 2, 1, fp);
				if (IsBigEndian())
				{
					BlueMask = FlipWORD(BlueMask);
				}
				SafeFread((char*)&TempMaskWORD, 2, 1, fp);
			}

			// read and skip any meta data

			if (BytesToSkip > 0)
			{
				unsigned char* TempSkipBYTE;
				TempSkipBYTE = new unsigned char[BytesToSkip];
				SafeFread((char*)TempSkipBYTE, BytesToSkip, 1, fp);
				delete[] TempSkipBYTE;
			}

			// determine the red, green and blue shifts

			int GreenShift = 0;
			unsigned short TempShiftWORD = GreenMask;
			while (TempShiftWORD > 31)
			{
				TempShiftWORD = TempShiftWORD >> 1; GreenShift++;
			}
			int BlueShift = 0;
			TempShiftWORD = BlueMask;
			while (TempShiftWORD > 31)
			{
				TempShiftWORD = TempShiftWORD >> 1; BlueShift++;
			}
			int RedShift = 0;
			TempShiftWORD = RedMask;
			while (TempShiftWORD > 31)
			{
				TempShiftWORD = TempShiftWORD >> 1; RedShift++;
			}

			// read the actual pixels

			for (j = Height - 1; j >= 0; j--)
			{
				i = 0;
				int ReadNumber = 0;
				while (ReadNumber < DataBytes)
				{
					unsigned short TempWORD;
					SafeFread((char*)&TempWORD, 2, 1, fp);
					if (IsBigEndian())
					{
						TempWORD = FlipWORD(TempWORD);
					}
					ReadNumber += 2;

					unsigned short Red = RedMask & TempWORD;
					unsigned short Green = GreenMask & TempWORD;
					unsigned short Blue = BlueMask & TempWORD;

					unsigned char BlueBYTE = (unsigned char)8 * (Blue >> BlueShift);
					unsigned char GreenBYTE = (unsigned char)8 * (Green >> GreenShift);
					unsigned char RedBYTE = (unsigned char)8 * (Red >> RedShift);

					(Pixels[i][j]).Red = RedBYTE;
					(Pixels[i][j]).Green = GreenBYTE;
					(Pixels[i][j]).Blue = BlueBYTE;

					i++;
				}
				ReadNumber = 0;
				while (ReadNumber < PaddingBytes)
				{
					unsigned char TempBYTE;
					SafeFread((char*)&TempBYTE, 1, 1, fp);
					ReadNumber++;
				}
			}

		}

		fclose(fp);
		return true;
	}

	RGBApixel GetColor(int ColorNumber)
	{
		RGBApixel Output;
		Output.Red = 255;
		Output.Green = 255;
		Output.Blue = 255;
		Output.Alpha = 0;

		using namespace std;
		if (BitDepth != 1 && BitDepth != 4 && BitDepth != 8)
		{
			return Output;
		}
		if (!Colors)
		{
			return Output;
		}
		if (ColorNumber >= GetNumberOfColors())
		{
			return Output;
		}
		Output = Colors[ColorNumber];
		return Output;
	}

	bool SetColor(int ColorNumber, RGBApixel NewColor)
	{
		if (BitDepth != 1 && BitDepth != 4 && BitDepth != 8)
		{
			return false;
		}
		if (!Colors)
		{
			return false;
		}
		if (ColorNumber >= GetNumberOfColors())
		{
			return false;
		}
		Colors[ColorNumber] = NewColor;
		return true;
	}

private:
	int BitDepth;
	int Width;
	int Height;
	RGBApixel** Pixels;
	RGBApixel* Colors;
	int XPelsPerMeter;
	int YPelsPerMeter;

	unsigned char* MetaData1;
	int SizeOfMetaData1;
	unsigned char* MetaData2;
	int SizeOfMetaData2;

	bool Read32bitRow(unsigned char* Buffer, int BufferSize, int Row)
	{
		int i;
		if (Width * 4 > BufferSize)
		{
			return false;
		}
		for (i = 0; i < Width; i++)
		{
			memcpy((char*)&(Pixels[i][Row]), (char*)Buffer + 4 * i, 4);
		}
		return true;
	}

	bool Read24bitRow(unsigned char* Buffer, int BufferSize, int Row)
	{
		int i;
		if (Width * 3 > BufferSize)
		{
			return false;
		}
		for (i = 0; i < Width; i++)
		{
			memcpy((char*)&(Pixels[i][Row]), Buffer + 3 * i, 3);
		}
		return true;
	}

	bool Read8bitRow(unsigned char* Buffer, int BufferSize, int Row)
	{
		int i;
		if (Width > BufferSize)
		{
			return false;
		}
		for (i = 0; i < Width; i++)
		{
			int Index = Buffer[i];
			*(this->operator()(i, Row)) = GetColor(Index);
		}
		return true;
	}

	bool Read4bitRow(unsigned char* Buffer, int BufferSize, int Row)
	{
		int Shifts[2] = { 4  ,0 };
		int Masks[2] = { 240,15 };

		int i = 0;
		int j;
		int k = 0;
		if (Width > 2 * BufferSize)
		{
			return false;
		}
		while (i < Width)
		{
			j = 0;
			while (j < 2 && i < Width)
			{
				int Index = (int)((Buffer[k] & Masks[j]) >> Shifts[j]);
				*(this->operator()(i, Row)) = GetColor(Index);
				i++; j++;
			}
			k++;
		}
		return true;
	}

	bool Read1bitRow(unsigned char* Buffer, int BufferSize, int Row)
	{
		int Shifts[8] = { 7  ,6 ,5 ,4 ,3,2,1,0 };
		int Masks[8] = { 128,64,32,16,8,4,2,1 };

		int i = 0;
		int j;
		int k = 0;

		if (Width > 8 * BufferSize)
		{
			return false;
		}
		while (i < Width)
		{
			j = 0;
			while (j < 8 && i < Width)
			{
				int Index = (int)((Buffer[k] & Masks[j]) >> Shifts[j]);
				*(this->operator()(i, Row)) = GetColor(Index);
				i++; j++;
			}
			k++;
		}
		return true;
	}

	bool Write32bitRow(unsigned char* Buffer, int BufferSize, int Row)
	{
		int i;
		if (Width * 4 > BufferSize)
		{
			return false;
		}
		for (i = 0; i < Width; i++)
		{
			memcpy((char*)Buffer + 4 * i, (char*)&(Pixels[i][Row]), 4);
		}
		return true;
	}

	bool Write24bitRow(unsigned char* Buffer, int BufferSize, int Row)
	{
		int i;
		if (Width * 3 > BufferSize)
		{
			return false;
		}
		for (i = 0; i < Width; i++)
		{
			memcpy((char*)Buffer + 3 * i, (char*)&(Pixels[i][Row]), 3);
		}
		return true;
	}

	bool Write8bitRow(unsigned char* Buffer, int BufferSize, int Row)
	{
		int i;
		if (Width > BufferSize)
		{
			return false;
		}
		for (i = 0; i < Width; i++)
		{
			Buffer[i] = FindClosestColor(Pixels[i][Row]);
		}
		return true;
	}

	bool Write4bitRow(unsigned char* Buffer, int BufferSize, int Row)
	{
		int PositionWeights[2] = { 16,1 };

		int i = 0;
		int j;
		int k = 0;
		if (Width > 2 * BufferSize)
		{
			return false;
		}
		while (i < Width)
		{
			j = 0;
			int Index = 0;
			while (j < 2 && i < Width)
			{
				Index += (PositionWeights[j] * (int)FindClosestColor(Pixels[i][Row]));
				i++; j++;
			}
			Buffer[k] = (unsigned char)Index;
			k++;
		}
		return true;
	}

	bool Write1bitRow(unsigned char* Buffer, int BufferSize, int Row)
	{
		int PositionWeights[8] = { 128,64,32,16,8,4,2,1 };

		int i = 0;
		int j;
		int k = 0;
		if (Width > 8 * BufferSize)
		{
			return false;
		}
		while (i < Width)
		{
			j = 0;
			int Index = 0;
			while (j < 8 && i < Width)
			{
				Index += (PositionWeights[j] * (int)FindClosestColor(Pixels[i][Row]));
				i++; j++;
			}
			Buffer[k] = (unsigned char)Index;
			k++;
		}
		return true;
	}

	unsigned char FindClosestColor(RGBApixel& input)
	{
		int i = 0;
		int NumberOfColors = GetNumberOfColors();
		unsigned char BestI = 0;
		int BestMatch = 999999;

		while (i < NumberOfColors)
		{
			RGBApixel Attempt = GetColor(i);
			int TempMatch = IntSquare((int)Attempt.Red - (int)input.Red)
				+ IntSquare((int)Attempt.Green - (int)input.Green)
				+ IntSquare((int)Attempt.Blue - (int)input.Blue);
			if (TempMatch < BestMatch)
			{
				BestI = (unsigned char)i; BestMatch = TempMatch;
			}
			if (BestMatch < 1)
			{
				i = NumberOfColors;
			}
			i++;
		}
		return BestI;
	}

	bool CreateStandardColorTable()
	{
		if (BitDepth != 1 && BitDepth != 4 && BitDepth != 8)
		{
			return false;
		}

		if (BitDepth == 1)
		{
			int i;
			for (i = 0; i < 2; i++)
			{
				Colors[i].Red = i * 255;
				Colors[i].Green = i * 255;
				Colors[i].Blue = i * 255;
				Colors[i].Alpha = 0;
			}
			return true;
		}

		if (BitDepth == 4)
		{
			int i = 0;
			int j, k, ell;

			// simplify the code for the first 8 colors
			for (ell = 0; ell < 2; ell++)
			{
				for (k = 0; k < 2; k++)
				{
					for (j = 0; j < 2; j++)
					{
						Colors[i].Red = j * 128;
						Colors[i].Green = k * 128;
						Colors[i].Blue = ell * 128;
						i++;
					}
				}
			}

			// simplify the code for the last 8 colors
			for (ell = 0; ell < 2; ell++)
			{
				for (k = 0; k < 2; k++)
				{
					for (j = 0; j < 2; j++)
					{
						Colors[i].Red = j * 255;
						Colors[i].Green = k * 255;
						Colors[i].Blue = ell * 255;
						i++;
					}
				}
			}

			// overwrite the duplicate color
			i = 8;
			Colors[i].Red = 192;
			Colors[i].Green = 192;
			Colors[i].Blue = 192;

			for (i = 0; i < 16; i++)
			{
				Colors[i].Alpha = 0;
			}
			return true;
		}

		if (BitDepth == 8)
		{
			int i = 0;
			int j, k, ell;

			// do an easy loop, which works for all but colors 
			// 0 to 9 and 246 to 255
			for (ell = 0; ell < 4; ell++)
			{
				for (k = 0; k < 8; k++)
				{
					for (j = 0; j < 8; j++)
					{
						Colors[i].Red = j * 32;
						Colors[i].Green = k * 32;
						Colors[i].Blue = ell * 64;
						Colors[i].Alpha = 0;
						i++;
					}
				}
			}

			// now redo the first 8 colors  
			i = 0;
			for (ell = 0; ell < 2; ell++)
			{
				for (k = 0; k < 2; k++)
				{
					for (j = 0; j < 2; j++)
					{
						Colors[i].Red = j * 128;
						Colors[i].Green = k * 128;
						Colors[i].Blue = ell * 128;
						i++;
					}
				}
			}

			// overwrite colors 7, 8, 9
			i = 7;
			Colors[i].Red = 192;
			Colors[i].Green = 192;
			Colors[i].Blue = 192;
			i++; // 8
			Colors[i].Red = 192;
			Colors[i].Green = 220;
			Colors[i].Blue = 192;
			i++; // 9
			Colors[i].Red = 166;
			Colors[i].Green = 202;
			Colors[i].Blue = 240;

			// overwrite colors 246 to 255 
			i = 246;
			Colors[i].Red = 255;
			Colors[i].Green = 251;
			Colors[i].Blue = 240;
			i++; // 247
			Colors[i].Red = 160;
			Colors[i].Green = 160;
			Colors[i].Blue = 164;
			i++; // 248
			Colors[i].Red = 128;
			Colors[i].Green = 128;
			Colors[i].Blue = 128;
			i++; // 249
			Colors[i].Red = 255;
			Colors[i].Green = 0;
			Colors[i].Blue = 0;
			i++; // 250
			Colors[i].Red = 0;
			Colors[i].Green = 255;
			Colors[i].Blue = 0;
			i++; // 251
			Colors[i].Red = 255;
			Colors[i].Green = 255;
			Colors[i].Blue = 0;
			i++; // 252
			Colors[i].Red = 0;
			Colors[i].Green = 0;
			Colors[i].Blue = 255;
			i++; // 253
			Colors[i].Red = 255;
			Colors[i].Green = 0;
			Colors[i].Blue = 255;
			i++; // 254
			Colors[i].Red = 0;
			Colors[i].Green = 255;
			Colors[i].Blue = 255;
			i++; // 255
			Colors[i].Red = 255;
			Colors[i].Green = 255;
			Colors[i].Blue = 255;

			return true;
		}
		return true;
	}

	static double Square(double number)
	{
		return number * number;
	}

	static int IntSquare(int number)
	{
		return number * number;
	}

	static int IntPow(int base, int exponent)
	{
		int i;
		int output = 1;
		for (i = 0; i < exponent; i++)
		{
			output *= base;
		}
		return output;
	}

	static bool SafeFread(char* buffer, int size, int number, FILE* fp)
	{
		int ItemsRead;
		if (feof(fp))
		{
			return false;
		}
		ItemsRead = (int)fread(buffer, size, number, fp);
		if (ItemsRead < number)
		{
			return false;
		}
		return true;
	}
};