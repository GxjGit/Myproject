#include "MyQtClass.h"


MyFileDialog::MyFileDialog( QWidget *parent/* = 0*/,
	const QString &caption/* = QString()*/,
	const QString &directory/* = QString()*/,
	const QString &filter/* = QString()*/,
	const bool previewInfo/* = false */)
	:QFileDialog(parent)
{
	this->setWindowTitle(caption);
	this->setFilter(filter);
	if ( !history().isEmpty() )
	{
		QString mm = QFileInfo(history().front()).absolutePath();
		setDirectory(mm);
	}//if ( !history().isEmpty() )
	if ( previewInfo )
	{
		createPreviewLabel();
	}//if ( previewInfo )
}

QString MyFileDialog::getFileNameAsOpen()
{
	QString fileName;
	setAcceptMode(AcceptOpen);
	if ( this->exec() )
	{
		QStringList files = selectedFiles();
		if ( !files.isEmpty() )
		{
			fileName =  files.back();
			setHistory(files);
		}
	}//if ( this->exec() )
	return fileName;
}

QString MyFileDialog::getFileNameAsSave()
{
	QString fileName;
	setAcceptMode(AcceptSave);
	setConfirmOverwrite(true);
	if ( this->exec() )
	{
		QStringList files = selectedFiles();
		if ( !files.isEmpty() )
		{
			fileName =  files.back();
			setHistory(files);
		}
	}//if ( this->exec() )
	return fileName;
}

void MyFileDialog::accept()
{
	if ( AcceptOpen == this->acceptMode() )
	{
		QDialog::accept();
		return;
	}
	QString fileName;
	QString ext = this->selectedFilter();
	int cont = ext.size();
	int s = ext.indexOf(".");
	int e = ext.lastIndexOf(")");
	ext = ext.right(cont-s-1);
	ext = ext.left(e-s-1);
	setDefaultSuffix(ext);
	QStringList files = selectedFiles();
	if ( !files.isEmpty() )
	{
		fileName =  files.back();
		if ( QFile::exists(fileName) )
		{
			int rtnflag = QMessageBox::warning(this,QObject::tr("Save File"),
				QObject::tr("%1 already exists. \nDo you want to replace it?").arg(QFileInfo(fileName).fileName()),
				QMessageBox::Yes,QMessageBox::No);
			if(rtnflag== QMessageBox::Yes)
			{
				QDialog::accept();
			}
			else if ( rtnflag == QMessageBox::No)
			{
				return ;
			}
		}
	}
	QDialog::accept();
}

void MyFileDialog::previewDisplay(const QString & str)
{
	
}

void MyFileDialog::createPreviewLabel()
{

}