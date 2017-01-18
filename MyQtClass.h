
#include <QtCore>
#include <QtGui>
#include <QtXml>

class MyFileDialog : public QFileDialog
{
	Q_OBJECT
public:
	MyFileDialog( QWidget *parent = 0,
		const QString &caption = QString(),
		const QString &directory = QString(),
		const QString &filter = QString(),
		const bool previewInfo = false );

	QString getFileNameAsOpen();

	QString getFileNameAsSave();
protected:
	virtual void accept();
private:
	void createPreviewLabel();
private:
	QLabel *m_pPreviewLabel;

	private slots:
		void previewDisplay(const QString & str );


};//end MyFileDialog